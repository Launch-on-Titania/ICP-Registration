import numpy as np
import open3d as o3d
import sys
import os
from sklearn.neighbors import KDTree

def nearest_search(source, target):
    indices = np.zeros(source.shape[0])
    distances = np.zeros(source.shape[0])
    for i, s in enumerate(source):
        distance_min = np.inf
        for j, t in enumerate(target):
            distance_element = np.linalg.norm(s - t)
            if distance_element < distance_min:
                distance_min = distance_element
                indices[i] = j
                distances[i] = distance_element
    return distances, indices

def icp(source, target, sample_num=500, max_iteration=80, tolerance=1e-6, Off_State_kdtree=False, Down_Sample=True):
    source_icp = np.ones((4, source.shape[0]))
    target_icp = np.ones((4, target.shape[0]))
    # notice the format: [coordinate * data + whole 1] 4 * points
    source_icp[0:3, :] = np.copy(source.T)
    target_icp[0:3, :] = np.copy(target.T)

    # initial the error
    error_pre = np.inf
    # iteration
    for i in range(max_iteration):
        print("No.", i, i / max_iteration * 100, "%")
        # find nearest points between source & target
        if Off_State_kdtree:
            if Down_Sample:
                # dawn sample process
                source_sample = source[np.random.choice(source.shape[0], sample_num, replace=False)]
                target_sample = target[np.random.choice(target.shape[0], sample_num, replace=False)]
                source_tree = KDTree(target_sample)
                distances, indices = source_tree.query(source_sample, k=1)
            else:
                source_tree = KDTree(target)
                distances, indices = source_tree.query(source, k=1)
        else:
            if Down_Sample:
                # dawn sample process
                source_sample = source[np.random.choice(source.shape[0], sample_num, replace=False)]
                target_sample = target[np.random.choice(target.shape[0], sample_num, replace=False)]
                distances, indices = nearest_search(source_sample, target_sample)
                distances = distances.reshape(-1, 1)
                indices = indices.reshape(-1, 1)
                indices = indices.astype(int)
            else:
                distances, indices = nearest_search(source, target)
                distances = distances.reshape(-1, 1)
                indices = indices.reshape(-1, 1)
                indices = indices.astype(int)

        # find the reliable transformation
        if Down_Sample:
            source_mean = np.mean(source_sample, axis=0)
            target_mean = np.mean(target_sample[indices], axis=0)
            source_regress = source_sample - source_mean  # [x_regress, y_regress, z_regress] data*3
            target_regress = target_sample[indices] - target_mean
        else:
            source_mean = np.mean(source, axis=0)
            target_mean = np.mean(target[indices], axis=0)
            source_regress = source - source_mean  # [x_regress, y_regress, z_regress] data*3
            target_regress = target[indices] - target_mean

        # define covariance matrix
        H = np.dot(source_regress.T, target_regress[:, 0, :])
        U, Sig, Vt = np.linalg.svd(H)  # SVD divide 3*3 1*3 3*3

        # compute the rotation
        R = np.dot(Vt.T, U.T)  # 3*3
        # when R less than zero, special reflection case
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = np.dot(Vt.T, U.T)

        # compute the translation
        t = target_mean - np.dot(source_mean, R)

        # combine the transformation matrix
        T = np.identity(4)  # build the cube matrix in diagram is 1
        T[0:3, 0:3] = R
        T[0:3, 3] = t

        # transform the source clouds
        source_icp = np.dot(T, source_icp)
        source = source_icp[0:3, :].T

        # check error
        error_mean = np.sum(distances) / distances.size
        print("Error:", error_mean)
        if np.abs(error_mean) < 0.3:
            break
        if np.abs(error_mean-error_pre) < tolerance:
            break
        error_pre = error_mean

    return T, source


def load_point_clouds(path):
    # Get the list of files in the directory
    files = os.listdir(path)
    files.sort()
    # Load each point cloud and store it in a list
    point_clouds = []

    for file in files:
        if file.endswith('.xyz'):
            print("reload, ok", file)
            point_cloud = np.loadtxt(os.path.join(path, file))
            point_clouds.append(point_cloud)

    return point_clouds


if __name__ == "__main__":
    target_reveal = np.loadtxt("sframe10.xyz")
    point_clouds = load_point_clouds('/home/arno/PycharmProjects/pythonProject')
    outputs = []
    num_file = 20
    for i in range(num_file):
        print("frame:", i)
        if i < (num_file-1):
            source = point_clouds[i + 1]
        else:
            source = point_clouds[0]
        # target = np.loadtxt("frame1.xyz")
        target = np.loadtxt("sframe10.xyz")
        Trans, output = icp(source, target)
        outputs.append(output)

    # creat Open3D object
    result_pcd = []
    source_pcd = o3d.geometry.PointCloud()
    target_pcd = o3d.geometry.PointCloud()
    for i in range(num_file):
        output_pcd = o3d.geometry.PointCloud()
        result_pcd.append(output_pcd)
        result_pcd[i].points = o3d.utility.Vector3dVector(outputs[i])
        result_pcd[i].paint_uniform_color([0, 0.851, 0.929])

    # set the pointclouds coordinate
    # source_pcd.points = o3d.utility.Vector3dVector(source)
    target_pcd.points = o3d.utility.Vector3dVector(target_reveal)
    target_pcd.paint_uniform_color([1, 0, 0])

    # creat visualization windows
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    # vis.add_geometry(source_pcd, reset_bounding_box=True)
    vis.add_geometry(target_pcd, reset_bounding_box=True)
    for i in range(num_file):
        vis.add_geometry(result_pcd[i], reset_bounding_box=True)

    # reveal the window
    option = vis.get_render_option()
    option.point_size = 1.5
    vis.run()
    vis.destroy_window()