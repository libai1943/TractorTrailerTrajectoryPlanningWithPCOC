% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function GenerateObstacleVertexes(gamma)
if (gamma > 1)
    gamma = 1;
elseif (gamma < 0)
    gamma = 0;
end

warning off
global obstacle_vertexes_
obstacle_vertexes2_ = obstacle_vertexes_;
number_of_obstacles = size(obstacle_vertexes2_, 2);

for ii = 1 : number_of_obstacles
    cur_obs = obstacle_vertexes2_{ii};
    x = cur_obs.x;
    y = cur_obs.y;
    Nv = length(x);
    center_x = mean(x(1:(Nv-1)));
    center_y = mean(y(1:(Nv-1)));
    err_x = x - center_x;
    err_y = y - center_y;
    cur_obs.x = center_x + err_x.* gamma;
    cur_obs.y = center_y + err_y.* gamma;
    obstacle_vertexes2_{ii} = cur_obs;
end

delete('Current_vertex');
fid = fopen('Current_vertex', 'w');
for ii = 1 : number_of_obstacles % From the first obstacle to the last
    cur_obs = obstacle_vertexes2_{ii};
    x = cur_obs.x;
    y = cur_obs.y;
    for jj = 1 : 4
        fprintf(fid,'%g %g %g %f \r\n', ii, jj, 1, x(jj));
        fprintf(fid,'%g %g %g %f \r\n', ii, jj, 2, y(jj));
    end
end
fclose(fid);

delete('Number_obstacle');
fid = fopen('Number_obstacle', 'w');
fprintf(fid,' %g\r\n', number_of_obstacles);
fclose(fid);

Area = [];
for ii = 1 : number_of_obstacles
    current_obstacle = obstacle_vertexes2_{ii};
    Area = [Area, CalculatePolygonArea(current_obstacle)];
end

delete('Area');
fid = fopen('Area', 'w');
for ii = 1 : number_of_obstacles
    fprintf(fid,'%g %g\r\n', ii, Area(ii));
end
fclose(fid);