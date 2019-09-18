% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function idx = Convert2DConfigToIndex(config)
global hybrid_astar_ planning_scale_
ind1 = ceil((config(1) - planning_scale_.xmin) / hybrid_astar_.resolution_x) + 1;
ind2 = ceil((config(2) - planning_scale_.ymin) / hybrid_astar_.resolution_y) + 1;

if (ind1 > hybrid_astar_.num_nodes_x)
    ind1 = hybrid_astar_.num_nodes_x;
    disp('Generate A * line 104 contains errors');
elseif (ind1 < 1)
    ind1 = 1;
    disp('Generate A * line 104 contains errors');
end
if (ind2 > hybrid_astar_.num_nodes_y)
    ind2 = hybrid_astar_.num_nodes_y;
    disp('Generate A * line 104 contains errors');
elseif (ind2 < 1)
    ind2 = 1;
    disp('Generate A * line 104 contains errors');
end
idx = [ind1, ind2];
end