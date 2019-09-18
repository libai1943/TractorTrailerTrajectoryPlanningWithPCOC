% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function [length1, err2to4] = GenerateRsPathForH(node)
global vehicle_physics_ hybrid_astar_
global nominal_goal_config_
global vehicle_boundary_value_configs_
startPose = [node(1), node(2), node(3)];
goalPose = [nominal_goal_config_(1), nominal_goal_config_(2), nominal_goal_config_(3)];
reedsConnObj = robotics.ReedsSheppConnection('MinTurningRadius', vehicle_physics_.min_turning_radius);
reedsConnObj.ReverseCost = hybrid_astar_.penalty_multiplier_for_reversing;
[pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
path_length = pathSegObj{1}.Length;
poses = interpolate(pathSegObj{1},[0:0.5:path_length]);
x1 = poses(:,1)';
y1 = poses(:,2)';
theta1 = poses(:,3)';
[~, ~, ~, theta2, theta3, theta4] = Fulfill6DimFrom3Dim(x1, y1, theta1, node(4), node(5), node(6));
[x2, y2, x3, y3, x4, y4] = ExpandFrom6DimToXY(x1, y1, theta1, theta2, theta3, theta4);
length1 = path_length;
err2 = [x2(end), y2(end)] - [vehicle_boundary_value_configs_.norminal_x2_end, vehicle_boundary_value_configs_.norminal_y2_end];
err3 = [x3(end), y3(end)] - [vehicle_boundary_value_configs_.norminal_x3_end, vehicle_boundary_value_configs_.norminal_y3_end];
err4 = [x4(end), y4(end)] - [vehicle_boundary_value_configs_.norminal_x4_end, vehicle_boundary_value_configs_.norminal_y4_end];
err2to4 = norm(err2) + norm(err3) + norm(err4);