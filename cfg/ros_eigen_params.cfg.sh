#!/usr/bin/env bash
echo "
# Path to config file for robot hand geometry
hand_geometry_filename = 0

# Path to config file for volume and image geometry
image_geometry_filename = 0

# ==== Robot Hand Geometry ====
#   finger_width: the width of the finger
#   outer_diameter: the diameter of the robot hand (= maximum aperture + 2 * finger width)
#   hand_depth: the finger length (measured from hand base to finger tip)
#   hand_height: the height of the hand
#   init_bite: the minimum amount of the object to be covered by the hand
finger_width = 0.01
hand_outer_diameter = 0.12
hand_depth = 0.06
hand_height = 0.02
init_bite = 0.01

# ==== Grasp Descriptor ====
#   volume_width: the width of the cube inside the robot hand
#   volume_depth: the depth of the cube inside the robot hand
#   volume_height: the height of the cube inside the robot hand
#   image_size: the size of the image (width and height; image is square)
#   image_num_channels: the number of image channels
volume_width = 0.10
volume_depth = 0.06
volume_height = 0.02
image_size = 60
image_num_channels = 15

# (OpenVINO) Path to directory that contains neural network parameters
weights_file = $(pwd)/../models/lenet/15channels/params/

# Preprocessing of point cloud
#   voxelize: if the cloud gets voxelized/downsampled
#   remove_outliers: if statistical outliers are removed from the cloud (used to remove noise)
#   workspace: the workspace of the robot (dimensions of a cube centered at origin of point cloud)
#   camera_position: the position of the camera from which the cloud was taken
#   sample_above_plane: only draws samples which do not belong to the table plane
voxelize = 1
remove_outliers = 0
workspace = -1.0 1.0 -1.0 1.0 -1.0 1.0
camera_position = 0 0 0
sample_above_plane = 0

# Grasp candidate generation
#   num_samples: the number of samples to be drawn from the point cloud
#   num_threads: the number of CPU threads to be used
#   nn_radius: the radius for the neighborhood search
#   num_orientations: the number of robot hand orientations to evaluate
#   rotation_axes: the axes about which the point neighborhood gets rotated
num_samples = 500
num_threads = 4
nn_radius = 0.01
num_orientations = 8
num_finger_placements = 10
hand_axes = 2
deepen_hand = 1

# Filtering of candidates
#   min_aperture: the minimum gripper width
#   max_aperture: the maximum gripper width
#   workspace_grasps: dimensions of a cube centered at origin of point cloud; should be smaller than <workspace>
min_aperture = 0.0
max_aperture = 0.85
workspace_grasps = -1 1 -1 1 -1 1

# Filtering of candidates based on their approach direction
#   filter_approach_direction: turn filtering on/off
#   direction: the direction to compare against
#   angle_thresh: angle in radians above which grasps are filtered
filter_approach_direction = 0
direction = 1 0 0
thresh_rad = 2.0

# Clustering of grasps
#   min_inliers: minimum number of inliers per cluster; set to 0 to turn off clustering
min_inliers = 1

# Grasp selection
#   num_selected: number of selected grasps (sorted by score)
num_selected = 100

# Visualization
#   plot_normals: plot the surface normals
#   plot_samples: plot the samples
#   plot_candidates: plot the grasp candidates
#   plot_filtered_candidates: plot the grasp candidates which remain after filtering
#   plot_valid_grasps: plot the candidates that are identified as valid grasps
#   plot_clustered_grasps: plot the grasps that after clustering
#   plot_selected_grasps: plot the selected grasps (final output)
plot_normals = 0
plot_samples = 0
plot_candidates = 0
plot_filtered_candidates = 0
plot_valid_grasps = 0
plot_clustered_grasps = 0
plot_selected_grasps = 1
"
