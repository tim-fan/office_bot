-- Config for running office bot in localisation only mode

include "red_bot.lua"

TRAJECTORY_BUILDER.pure_localization = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120 
TRAJECTORY_BUILDER_2D.submaps.resolution = 0.05
--POSE_GRAPH.optimize_every_n_nodes = 10
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8
MAP_BUILDER.pose_graph.constraint_builder.min_score = 0.75
return options
