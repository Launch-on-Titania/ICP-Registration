# ICP_registration_algorithm
ICP implement by source code (no off-the-shelf ICP function); visualized by open3d

## Requirement (Python)

- Numpy
- Open3D

## Achievement

In this assignment, I first implemented registering a single point cloud to the target point cloud.

Then extended it to register the entire sequence of point clouds to the target.

### Setup: input Pointclouds

The format of pointclouds is `.xyz`, which means the given pointclouds only hold the xyz coordinate info. Based on the given the sequenceof pointclouds in 20 frames. By using:

    point_cloud = np.loadtxt(os.path.join(path, file))

Achieved the loading of pointclouds, then apply the data to the ICP algorithm.

### ICP Achieve

In this part, it shows the simple implement of ICP algriothm.

    def icp(source, target, sample_num=500, 
    max_iteration=80, tolerance=1e-6, 
    Off_State_kdtree=False, Down_Sample=True):

The following is my achievement pipeline：

$R^{\ast}, t^{\ast} = \mathop{\arg\min}_{R, t} \frac{1}{|P_s|} \sum_{i=1}^{|P_s|} || p_t^i - (R \cdot p_s^i + t) ||^2$

- Initial the paras of the icp function:
  
  Included source & target pointclouds, max_iteration, dawn_sample_points, and the torlerance of shutting the iteration.
- Uses KDtree or `nearest search`(*Set an appropriate value for downsampling and implement brute-force retrieval of the nearest Euclidean distance.*) to find the nearest neighbor points, gets the`dictance`&`indices`, which is used in compute the distances error.
  ***The distance and the indices are compute by function`def nearest_search(source, target)`***
- Down sample the given pointclouds, through the`sample_num`to accomplish the down sample.
- Find the reliable transformation: Rotation & Translation
  - Based on ICP algriothm, define the covariance matrix. By `SVD` division, compute the U & V matrix.

    $H=∑^{|Ps|}_{i=1}p^{i}_{s}p^{iT}_t=UΣV^T$

  - And point to point ICP's best `Rotation` is explained as:

    $R^{\ast}=VU^T$
  - Best `Translation` is explained as:
  
    $t^{\ast} = \bar p_t - R^{\ast} \bar p_s$
  - Combine the`Rotation` & `Translation` matrix into `Transformation`matrix.
- At the biginning, the related paras like `tolerance`&`max_iteration`are used to implement breaking out of iteration loops and obtaining the`Transformation`matrix within the allowed range of error.

## Result

In this part, it would given the result from ICP implement with the following sequence of pointclouds frames.

- Notice: `Blue` refers to the source pointclouds; `Red` refers to target pointclouds.

### Single Registration

At first, I achieve the basic registration in single frame. Set the target poinclouds as`frame1.xyz`&`frame11.xyz`

<img src="/result/single_unregistration.jpg" width = "80%" /> <img src="/result/single_registration.png" width = "80%" />

The former one is the unregistration situation, which is obviously unaligned.

Through the process of`T=icp(source, target)`, we get the last image, which is clearly achieved aligned and implement the registration algriothm.

Tn this part, I set the`frame1.xyz`as the target (groundtruth), and the `Transformation`matrix is as followed:

    [[ 9.99982229e-01, -5.93854481e-03,  5.25001814e-04, -5.70442354e-03],
    [ 5.93801473e-03,  9.99981864e-01,  1.00552318e-03, -4.54894182e-03],
    [-5.30963638e-04, -1.00238784e-03,  9.99999357e-01, -1.37084380e-03],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]

Through this matrix, it easily accomplished the registration between`frame1.xyz`(target)and`frame11.xyz`(source).
