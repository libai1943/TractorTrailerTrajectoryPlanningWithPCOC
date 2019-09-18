% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function obstacle_cell = GenerateObstacles()
global case_id
switch case_id
    case 1
        Nobs = 2;
        obstacle_cell = cell(1, Nobs);
        current_obstacle.x = [-20, -10, -5, -20, -20];
        current_obstacle.y = [-15, -15, 0, 5, -15];
        current_obstacle.A = CalculatePolygonArea(current_obstacle);
        obstacle_cell{1,1} = current_obstacle;
        current_obstacle.x = [0, 20, 20, -2, 0];
        current_obstacle.y = [-20, -20, -2, -2, -20];
        current_obstacle.A = CalculatePolygonArea(current_obstacle);
        obstacle_cell{1,2} = current_obstacle;
    case 2
        Nobs = 3;
        obstacle_cell = cell(1, Nobs);
        current_obstacle.x = [-12.8100  -11.5600  -15.1900  -16.4400 -12.8100];
        current_obstacle.y = [2.4200    1.0100   -2.2000   -0.7900   2.4200];
        current_obstacle.A = CalculatePolygonArea(current_obstacle);
        obstacle_cell{1,1} = current_obstacle;
        current_obstacle.x = [-5.8500   -4.3800   -2.0500   -3.5200 -5.8500];
        current_obstacle.y = [5.1200    5.9900    2.0800    1.2100 5.1200];
        current_obstacle.A = CalculatePolygonArea(current_obstacle);
        obstacle_cell{1,2} = current_obstacle;
        current_obstacle.x = [5.4700    5.3800   5-4.4700   5-4.3800   5.4700] - 1.0;
        current_obstacle.y = [-2.1800   -4.0600   -3.8200   -1.9400  -2.1800];
        current_obstacle.A = CalculatePolygonArea(current_obstacle);
        obstacle_cell{1,3} = current_obstacle;
    case 3
        Nobs = 2;
        obstacle_cell = cell(1, Nobs);
        current_obstacle.x = [-20 -7 -7 -20 -20];
        current_obstacle.y = [0 0 -20 -20 0];
        current_obstacle.A = CalculatePolygonArea(current_obstacle);
        obstacle_cell{1,1} = current_obstacle;
        current_obstacle.x = [-3 20 20 -3 -3];
        current_obstacle.y = [0 0 -20 -20 0];
        current_obstacle.A = CalculatePolygonArea(current_obstacle);
        obstacle_cell{1,2} = current_obstacle;
end
end