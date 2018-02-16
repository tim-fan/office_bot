-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "caddy.lua"

TRAJECTORY_BUILDER.pure_localization = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120 
TRAJECTORY_BUILDER_2D.submaps.resolution = 0.1
--POSE_GRAPH.optimize_every_n_nodes = 10
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8
MAP_BUILDER.pose_graph.constraint_builder.min_score = 0.75
return options
