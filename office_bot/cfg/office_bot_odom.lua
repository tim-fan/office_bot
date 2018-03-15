-- Config for running office bot solely for producing laser-odometry
-- Turns off global map optimisation

include "office_bot.lua"
-- options.map_frame = "cartographer_map" --trying to figure out how to prevent cartographer publishing localisation (want only odom published)
POSE_GRAPH.optimize_every_n_nodes = 0
return options
