% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function val = SpecifyGoalConfig()
global vehicle_boundary_value_configs_
val = [vehicle_boundary_value_configs_.norminal_x1_end, vehicle_boundary_value_configs_.norminal_y1_end, ...
    vehicle_boundary_value_configs_.norminal_theta1_end, vehicle_boundary_value_configs_.norminal_theta2_end, ...
    vehicle_boundary_value_configs_.norminal_theta3_end, vehicle_boundary_value_configs_.norminal_theta4_end];