include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "xsens_frame",
  published_frame = "base_link",
  odom_frame = "cartographer_odom",
  publish_tracked_pose=true,
  publish_to_tf=true,
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}



--TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 50
--TRAJECTORY_BUILDER_3D.max_range = 4
MAP_BUILDER.use_trajectory_builder_3d = true



--Local SLAM
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 30
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 2e3
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1e1




MAP_BUILDER.num_background_threads = 10


--Global SLAM
--TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false
--POSE_GRAPH.optimize_every_n_nodes = 100

--POSE_GRAPH.constraint_builder.sampling_ratio = 2e3 --Increase use more nodes in finding the loop closure

--TRAJECTORY_BUILDER_3D.submaps.num_range_data = 1e8
--POSE_GRAPH.constraint_builder.sampling_ratio = 1e8
--POSE_GRAPH.optimize_every_n_nodes = 1e8

--POSE_GRAPH.constraint_builder.max_constraint_distance = 100
--POSE_GRAPH.optimization_problem.huber_scale = 5e2

--POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 --Increase to evaluate more loop closure candidates
--POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.45
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45 --Reduce to accept more (and potentially worse candidates as constraints).


return options