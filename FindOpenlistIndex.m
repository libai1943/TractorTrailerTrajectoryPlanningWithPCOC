% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function index = FindOpenlistIndex(openlist_, index0)
for index = 1 : size(openlist_,1)
    index_candidate = Convert6DimConfigToIndex(openlist_(index,1:6));
    if (index_candidate == index0)
        return;
    end
end