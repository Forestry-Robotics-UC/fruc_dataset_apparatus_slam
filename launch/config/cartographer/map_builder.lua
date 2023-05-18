include "pose_graph.lua"

MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = true,
  num_background_threads = 10,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
}