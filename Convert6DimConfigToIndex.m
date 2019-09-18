% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function idx = Convert6DimConfigToIndex(node)
global hybrid_astar_ planning_scale_
ind1 = ceil((node(1) - planning_scale_.xmin) / hybrid_astar_.resolution_x) + 1;
ind2 = ceil((node(2) - planning_scale_.ymin) / hybrid_astar_.resolution_y) + 1;
ind3 = ceil((RegulateAngle(node(3))) / hybrid_astar_.resolution_theta) + 1;
ind4 = ceil((RegulateAngle(node(4))) / hybrid_astar_.resolution_theta) + 1;
ind5 = ceil((RegulateAngle(node(5))) / hybrid_astar_.resolution_theta) + 1;
ind6 = ceil((RegulateAngle(node(5))) / hybrid_astar_.resolution_theta) + 1;
idx = [ind1, ind2, ind3, ind4, ind5, ind6];
end

function angle  = RegulateAngle(angle)
while (angle > 2 * pi + 0.000001)
    angle = angle - 2 * pi;
end
while (angle < - 0.000001)
    angle = angle + 2 * pi;
end
end