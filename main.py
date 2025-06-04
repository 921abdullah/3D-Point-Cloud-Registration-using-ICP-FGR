import open3d as o3d
import numpy as np
import time
import copy

# loading two sample point clouds from Open3D's test dataset
source = o3d.io.read_point_cloud(o3d.data.DemoICPPointClouds().paths[0])
target = o3d.io.read_point_cloud(o3d.data.DemoICPPointClouds().paths[1])

# downsampling and estimating normals to reduce computation and prepare for registration
def preprocess(pcd, voxel_size=0.05):
    # Downsample the point cloud
    pcd_down = pcd.voxel_down_sample(voxel_size)
    # Estimate normals to help with point-to-plane ICP
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    return pcd_down

# Visualize the aligned point clouds
def draw_registration_result(source, target, transformation):
    # Deep copy the clouds to preserve originals
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # Apply transformation to the source point cloud
    source_temp.transform(transformation)
    # Render both clouds
    o3d.visualization.draw_geometries([source_temp, target_temp])

# Run ICP (Iterative Closest Point) registration
def run_icp(source, target):
    print("\nRunning ICP...")
    threshold = 0.05  # Maximum correspondence point-pair distance
    trans_init = np.eye(4)  # Initial transformation (identity matrix)

    start = time.time()
    result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    end = time.time()

    # Display metrics
    print("ICP time: {:.4f} seconds".format(end - start))
    print("ICP Fitness:", result.fitness)
    print("ICP Inlier RMSE:", result.inlier_rmse)

    return result.transformation

# Run Fast Global Registration
def run_fgr(source, target, voxel_size=0.05):
    print("\nRunning Fast Global Registration...")

    # Preprocess both clouds
    source_down = preprocess(source, voxel_size)
    target_down = preprocess(target, voxel_size)

    # Extract Fast Point Feature Histograms (FPFH)
    def compute_fpfh(pcd):
        return o3d.pipelines.registration.compute_fpfh_feature(
            pcd,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)
        )

    source_fpfh = compute_fpfh(source_down)
    target_fpfh = compute_fpfh(target_down)

    # Define FGR parameters
    option = o3d.pipelines.registration.FastGlobalRegistrationOption(
        maximum_correspondence_distance=voxel_size * 1.5)

    start = time.time()
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, option)
    end = time.time()

    # Display metrics
    print("FGR time: {:.4f} seconds".format(end - start))
    print("FGR Fitness:", result.fitness)
    print("FGR Inlier RMSE:", result.inlier_rmse)

    return result.transformation

# ICP
source_pre = preprocess(source)
target_pre = preprocess(target)
T_icp = run_icp(source_pre, target_pre)
draw_registration_result(source, target, T_icp)

# FGR
T_fgr = run_fgr(source, target)
draw_registration_result(source, target, T_fgr)