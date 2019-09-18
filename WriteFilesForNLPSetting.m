% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function WriteFilesForNLPSetting()
global vehicle_geometrics_ vehicle_boundary_value_configs_
MM = vehicle_geometrics_.M;

delete('TNV');
fid = fopen('TNV', 'w');
fprintf(fid, '1  %f \r\n', vehicle_boundary_value_configs_.norminal_x1_end);
fprintf(fid, '2  %f \r\n', vehicle_boundary_value_configs_.norminal_y1_end);
fprintf(fid, '3  %f \r\n', vehicle_boundary_value_configs_.norminal_x2_end);
fprintf(fid, '4  %f \r\n', vehicle_boundary_value_configs_.norminal_y2_end);
fprintf(fid, '5  %f \r\n', vehicle_boundary_value_configs_.norminal_x3_end);
fprintf(fid, '6  %f \r\n', vehicle_boundary_value_configs_.norminal_y3_end);
fprintf(fid, '7  %f \r\n', vehicle_boundary_value_configs_.norminal_x4_end);
fprintf(fid, '8  %f \r\n', vehicle_boundary_value_configs_.norminal_y4_end);
fclose(fid);

delete('BVC');
fid = fopen('BVC', 'w');
fprintf(fid, '1  %f \r\n', vehicle_boundary_value_configs_.x1_init);
fprintf(fid, '2  %f \r\n', vehicle_boundary_value_configs_.y1_init);
fprintf(fid, '3  %f \r\n', 0);
fprintf(fid, '4  %f \r\n', 0);
fprintf(fid, '5  %f \r\n', vehicle_boundary_value_configs_.theta1_init);
fprintf(fid, '6  %f \r\n', vehicle_boundary_value_configs_.theta2_init);
fprintf(fid, '7  %f \r\n', vehicle_boundary_value_configs_.theta3_init);
fprintf(fid, '8  %f \r\n', vehicle_boundary_value_configs_.theta4_init);
fprintf(fid, '9  %f \r\n', vehicle_boundary_value_configs_.xc_end);
fprintf(fid, '10  %f \r\n', vehicle_boundary_value_configs_.yc_end);
fprintf(fid, '11  %f \r\n', vehicle_boundary_value_configs_.x_end_half_width);
fprintf(fid, '12  %f \r\n', vehicle_boundary_value_configs_.y_end_half_width);
fclose(fid);

delete('M');
fid = fopen('M', 'w');
fprintf(fid, '1  %f \r\n', MM(1));
fprintf(fid, '2  %f \r\n', MM(2));
fprintf(fid, '3  %f \r\n', MM(3));
fprintf(fid, '4  %f \r\n', MM(4));
fclose(fid);