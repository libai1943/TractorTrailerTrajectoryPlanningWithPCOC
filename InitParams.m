% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
global planning_scale_
planning_scale_.xmin = -30;
planning_scale_.xmax = 20;
planning_scale_.ymin = -20;
planning_scale_.ymax = 20;
planning_scale_.xhorizon = 50;
planning_scale_.yhorizon = 40;

global vehicle_geometrics_
vehicle_geometrics_.L_tractor_front_hang = 0.25;
vehicle_geometrics_.L_tractor_wheelbase = 1.5;
vehicle_geometrics_.L_tractor_rear_hang = 0.25;
vehicle_geometrics_.LHW = 1;
vehicle_geometrics_.M = [1.5, 1.5, 1.5, 1.5] .* off_axle_flag;
vehicle_geometrics_.L = [3.0, 3.0, 3.0, 3.0];
vehicle_geometrics_.L_trailer_front_hang = 1;
vehicle_geometrics_.L_trailer_rear_hang = 1;

global vehicle_physics_
vehicle_physics_.v_max = 1.5;
vehicle_physics_.phy_max = 0.7;
vehicle_physics_.a_max = 0.25;
vehicle_physics_.w_max = 0.5;
vehicle_physics_.min_turning_radius = vehicle_geometrics_.L_tractor_wheelbase / tan(vehicle_physics_.phy_max);

global vehicle_boundary_value_configs_
switch case_id
    case 1
        vehicle_boundary_value_configs_.x1_init = -15;
        vehicle_boundary_value_configs_.y1_init = -17.5;
        vehicle_boundary_value_configs_.theta1_init = 0;
        vehicle_boundary_value_configs_.theta2_init = 0;
        vehicle_boundary_value_configs_.theta3_init = 0;
        vehicle_boundary_value_configs_.theta4_init = 0;
        vehicle_boundary_value_configs_.v1_init = 0;
        vehicle_boundary_value_configs_.phy1_init = 0;
        vehicle_boundary_value_configs_.xc_end = 10;
        vehicle_boundary_value_configs_.yc_end = 0;
        vehicle_boundary_value_configs_.x_end_half_width = 10;
        vehicle_boundary_value_configs_.y_end_half_width = 1.5;
        vehicle_boundary_value_configs_.norminal_x1_end = 16;
        vehicle_boundary_value_configs_.norminal_y1_end = 0;
        vehicle_boundary_value_configs_.norminal_theta1_end = 0;
        vehicle_boundary_value_configs_.norminal_theta2_end = 0;
        vehicle_boundary_value_configs_.norminal_theta3_end = 0;
        vehicle_boundary_value_configs_.norminal_theta4_end = 0;
        [x2, y2, x3, y3, x4, y4] = ExpandFrom6DimToXY(vehicle_boundary_value_configs_.norminal_x1_end, vehicle_boundary_value_configs_.norminal_y1_end, vehicle_boundary_value_configs_.norminal_theta1_end, vehicle_boundary_value_configs_.norminal_theta2_end, vehicle_boundary_value_configs_.norminal_theta3_end, vehicle_boundary_value_configs_.norminal_theta4_end);
        vehicle_boundary_value_configs_.norminal_x2_end = x2;
        vehicle_boundary_value_configs_.norminal_y2_end = y2;
        vehicle_boundary_value_configs_.norminal_x3_end = x3;
        vehicle_boundary_value_configs_.norminal_y3_end = y3;
        vehicle_boundary_value_configs_.norminal_x4_end = x4;
        vehicle_boundary_value_configs_.norminal_y4_end = y4;
    case 2
        vehicle_boundary_value_configs_.x1_init = -10;
        vehicle_boundary_value_configs_.y1_init = 10;
        vehicle_boundary_value_configs_.theta1_init = 0;
        vehicle_boundary_value_configs_.theta2_init = 0;
        vehicle_boundary_value_configs_.theta3_init = 0;
        vehicle_boundary_value_configs_.theta4_init = 0;
        vehicle_boundary_value_configs_.v1_init = 0;
        vehicle_boundary_value_configs_.phy1_init = 0;
        vehicle_boundary_value_configs_.xc_end = -2;
        vehicle_boundary_value_configs_.yc_end = 0;
        vehicle_boundary_value_configs_.x_end_half_width = 10;
        vehicle_boundary_value_configs_.y_end_half_width = 1.25;
        vehicle_boundary_value_configs_.norminal_x1_end = 0;
        vehicle_boundary_value_configs_.norminal_y1_end = 0;
        vehicle_boundary_value_configs_.norminal_theta1_end = 0;
        vehicle_boundary_value_configs_.norminal_theta2_end = 0;
        vehicle_boundary_value_configs_.norminal_theta3_end = 0;
        vehicle_boundary_value_configs_.norminal_theta4_end = 0;
        [x2, y2, x3, y3, x4, y4] = ExpandFrom6DimToXY(vehicle_boundary_value_configs_.norminal_x1_end, vehicle_boundary_value_configs_.norminal_y1_end, vehicle_boundary_value_configs_.norminal_theta1_end, vehicle_boundary_value_configs_.norminal_theta2_end, vehicle_boundary_value_configs_.norminal_theta3_end, vehicle_boundary_value_configs_.norminal_theta4_end);
        vehicle_boundary_value_configs_.norminal_x2_end = x2;
        vehicle_boundary_value_configs_.norminal_y2_end = y2;
        vehicle_boundary_value_configs_.norminal_x3_end = x3;
        vehicle_boundary_value_configs_.norminal_y3_end = y3;
        vehicle_boundary_value_configs_.norminal_x4_end = x4;
        vehicle_boundary_value_configs_.norminal_y4_end = y4;
    case 3
        vehicle_boundary_value_configs_.x1_init = 16;
        vehicle_boundary_value_configs_.y1_init = 16;
        vehicle_boundary_value_configs_.theta1_init = pi/6;
        vehicle_boundary_value_configs_.theta2_init = pi/6;
        vehicle_boundary_value_configs_.theta3_init = pi/6;
        vehicle_boundary_value_configs_.theta4_init = pi/6;
        vehicle_boundary_value_configs_.v1_init = 0;
        vehicle_boundary_value_configs_.phy1_init = 0;
        vehicle_boundary_value_configs_.xc_end = -5;
        vehicle_boundary_value_configs_.yc_end = -11;
        vehicle_boundary_value_configs_.x_end_half_width = 1.25;
        vehicle_boundary_value_configs_.y_end_half_width = 9;
        vehicle_boundary_value_configs_.norminal_x1_end = -5;
        vehicle_boundary_value_configs_.norminal_y1_end = -3;
        vehicle_boundary_value_configs_.norminal_theta1_end = pi/2;
        vehicle_boundary_value_configs_.norminal_theta2_end = pi/2;
        vehicle_boundary_value_configs_.norminal_theta3_end = pi/2;
        vehicle_boundary_value_configs_.norminal_theta4_end = pi/2;
        [x2, y2, x3, y3, x4, y4] = ExpandFrom6DimToXY(vehicle_boundary_value_configs_.norminal_x1_end, vehicle_boundary_value_configs_.norminal_y1_end, vehicle_boundary_value_configs_.norminal_theta1_end, vehicle_boundary_value_configs_.norminal_theta2_end, vehicle_boundary_value_configs_.norminal_theta3_end, vehicle_boundary_value_configs_.norminal_theta4_end);
        vehicle_boundary_value_configs_.norminal_x2_end = x2;
        vehicle_boundary_value_configs_.norminal_y2_end = y2;
        vehicle_boundary_value_configs_.norminal_x3_end = x3;
        vehicle_boundary_value_configs_.norminal_y3_end = y3;
        vehicle_boundary_value_configs_.norminal_x4_end = x4;
        vehicle_boundary_value_configs_.norminal_y4_end = y4;
end

global hybrid_astar_
hybrid_astar_.resolution_x = 0.3;
hybrid_astar_.resolution_y = 0.3;
hybrid_astar_.resolution_theta = 0.5;
hybrid_astar_.num_nodes_x = ceil(planning_scale_.xhorizon / hybrid_astar_.resolution_x) + 1;
hybrid_astar_.num_nodes_y = ceil(planning_scale_.yhorizon / hybrid_astar_.resolution_x) + 1;
hybrid_astar_.num_nodes_theta = ceil(2 * pi / hybrid_astar_.resolution_theta) + 1;
hybrid_astar_.penalty_multiplier_for_reversing = 3;
hybrid_astar_.penalty_multiplier_for_drastic_direction_change = 5;
hybrid_astar_.multiplier_H = 3.0;
hybrid_astar_.num_iters_for_rs = 10;
hybrid_astar_.max_iter = 500;
hybrid_astar_.terminal_xy_neiborhood = 5.0;
hybrid_astar_.terminal_theta_neiborhood = 1.0;

global IPOPT_
IPOPT_.nfe = 101;
