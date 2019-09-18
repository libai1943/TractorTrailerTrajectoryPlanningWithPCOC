% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function is_close = Is6DNodeCloseToGoal(node_child)
global vehicle_boundary_value_configs_
global hybrid_astar_
is_close = 1;
err_xy = norm(node_child(1:2) - [vehicle_boundary_value_configs_.norminal_x1_end, vehicle_boundary_value_configs_.norminal_y1_end]);
if (err_xy > hybrid_astar_.terminal_xy_neiborhood)
    is_close = 0;
    return;
end
err_theta = max(abs(node_child(3:6) - [vehicle_boundary_value_configs_.norminal_theta1_end, vehicle_boundary_value_configs_.norminal_theta2_end, vehicle_boundary_value_configs_.norminal_theta3_end, vehicle_boundary_value_configs_.norminal_theta4_end]));
if (err_theta > hybrid_astar_.terminal_theta_neiborhood)
    is_close = 0;
    return;
end