-- Config for running office bot solely for producing laser-odometry
-- Targeted to red-robot, as red-robot has no odometry
-- Turns off global map optimisation

include "red_bot.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
return options
