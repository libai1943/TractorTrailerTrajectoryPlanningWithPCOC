% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function val = EvaluateFitness(node)
global vehicle_boundary_value_configs_
[x1, y1, theta1, theta2, theta3, theta4] = Fulfill6DimFrom3Dim(node(1), node(2), node(3), node(4), node(5), node(6));
[x2, y2, x3, y3, x4, y4] = ExpandFrom6DimToXY(x1, y1, theta1, theta2, theta3, theta4);
err1 = [x1(end), y1(end)] - [vehicle_boundary_value_configs_.norminal_x1_end, vehicle_boundary_value_configs_.norminal_y1_end];
err2 = [x2(end), y2(end)] - [vehicle_boundary_value_configs_.norminal_x2_end, vehicle_boundary_value_configs_.norminal_y2_end];
err3 = [x3(end), y3(end)] - [vehicle_boundary_value_configs_.norminal_x3_end, vehicle_boundary_value_configs_.norminal_y3_end];
err4 = [x4(end), y4(end)] - [vehicle_boundary_value_configs_.norminal_x4_end, vehicle_boundary_value_configs_.norminal_y4_end];
val = norm(err1) + 5 * norm(err2) + 25 * norm(err3) + 125 * norm(err4);