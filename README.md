## 3D-Point-Cloud-Registration-using-ICP-FGR 
When a LiDAR scanner (e.g on a car or robot) captures 3D data from different positions, each scan is in its own coordinate system. To combine these scans into a single 3D map, we must align (or “register”) them accurately.  This process is called Point Cloud Registration.

# ICP – Iterative Closest Point 
A local registration algorithm that aligns two 3D point clouds by minimizing the distance between their closest points.

Working:
1. Find Closest Points: For each point in the source cloud, find the closest point in the target cloud.
2. Estimate Transformation: Compute the best rigid transformation (rotation + translation) that minimizes the total distance between matched pairs.
3. Apply Transformation: Move the source cloud using the transformation.
4. Repeat: Iterate until the change becomes negligible (converges).

# FGR – Fast Global Registration 
FGR is a global registration algorithm that aligns point clouds by comparing their geometric features — not just point positions — and does not require a good initial guess.

Working:
1. Downsample the point clouds for speed.
2. Extract Features: Compute descriptors (like FPFH – Fast Point Feature Histograms) for each key point.
3. Match Features: Find feature correspondences between source and target.
4. Optimize Alignment: Estimate the best transformation using those correspondences.

# Objective 
Align two overlapping 3D point cloud scans using:
1. ICP (Iterative Closest Point) – For fine-tuning alignment when initial guess is close.
2. FGR (Fast Global Registration) – for fast, global matching based on features, without needing an initial guess.

Then compare both in terms of
1. Accuracy (Fitness and RMSE)
2. Speed
3. Visual result
