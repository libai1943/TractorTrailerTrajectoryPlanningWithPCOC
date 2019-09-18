% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function val = CalculateH(node)
global nominal_goal_config_
[length_xy, err2to4] = GenerateRsPathForH(node);
nonholonomic_without_collision_avoidance = length_xy + err2to4;

begin_config = [node(1),node(2)];
end_config = [nominal_goal_config_(1), nominal_goal_config_(2)];
holonomic_with_collision_avoidance = GenerateAStarPath(begin_config, end_config);

global hybrid_astar_
val = hybrid_astar_.multiplier_H * max(nonholonomic_without_collision_avoidance, holonomic_with_collision_avoidance);