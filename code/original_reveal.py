import numpy as np
import open3d as o3d
import os


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

point_clouds = load_point_clouds('/home/arno/PycharmProjects/pythonProject')
file_num = 19
# 创建Open3D点云对象
source_pcd = []
target_pcd = o3d.geometry.PointCloud()
for i in range(file_num):
    output_pcd = o3d.geometry.PointCloud()
    source_pcd.append(output_pcd)
    if i >= 9:
        source_pcd[i].points = o3d.utility.Vector3dVector(point_clouds[i+1])
    else:
        source_pcd[i].points = o3d.utility.Vector3dVector(point_clouds[i])
    source_pcd[i].paint_uniform_color([0, 0.851, 0.929])

# 设置点云对象的点坐标
# source_pcd.points = o3d.utility.Vector3dVector(source)
target_pcd.points = o3d.utility.Vector3dVector(point_clouds[9])
target_pcd.paint_uniform_color([1, 0, 0])

# 创建可视化窗口并添加点云对象
vis = o3d.visualization.Visualizer()
vis.create_window()
for i in range(file_num):
    vis.add_geometry(source_pcd[i], reset_bounding_box=True)
vis.add_geometry(target_pcd, reset_bounding_box=True)

option = vis.get_render_option()
option.point_size = 1.5

# 显示可视化窗口
vis.run()
vis.destroy_window()
