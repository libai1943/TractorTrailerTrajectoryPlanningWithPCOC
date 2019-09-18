% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function costmap = CreateCostmap()
global planning_scale_
xmin = planning_scale_.xmin;
ymin = planning_scale_.ymin;
global hybrid_astar_
resolution_x = hybrid_astar_.resolution_x;
resolution_y = hybrid_astar_.resolution_y;
NumGridsX = hybrid_astar_.num_nodes_x;
NumGridsY = hybrid_astar_.num_nodes_y;
costmap = zeros(NumGridsX, NumGridsY);
global obstacle_vertexes_

for ii = 1 : NumGridsX
    for jj = 1 : NumGridsY
        cur_x = xmin + (ii - 1) * resolution_x;
        cur_y = ymin + (jj - 1) * resolution_y;
        for kk = 1 : size(obstacle_vertexes_, 2)
            if (inpolygon(cur_x, cur_y, obstacle_vertexes_{kk}.x, obstacle_vertexes_{kk}.y) == 1)
                costmap(ii, jj) = 1;
                break;
            end
        end
    end
end
end