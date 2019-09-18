% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function [x1, y1, theta1, theta2, theta3, theta4] = GenerateRsPathForSearch(node)
global vehicle_physics_ hybrid_astar_
global nominal_goal_config_
reedsConnObj = robotics.ReedsSheppConnection('MinTurningRadius', vehicle_physics_.min_turning_radius);
reedsConnObj.ReverseCost = hybrid_astar_.penalty_multiplier_for_reversing;
[pathSegObj,~] = connect(reedsConnObj, node(1,1:3), nominal_goal_config_(1,1:3));
path_length = pathSegObj{1}.Length;
output_path_resolution = 0.25;
poses = interpolate(pathSegObj{1},[0:output_path_resolution:path_length]);
x = poses(:,1)';
y = poses(:,2)';
theta1 = poses(:,3)';
[x1, y1, theta1, theta2, theta3, theta4] = Fulfill6DimFrom3Dim(x, y, theta1, node(4), node(5), node(6));
end