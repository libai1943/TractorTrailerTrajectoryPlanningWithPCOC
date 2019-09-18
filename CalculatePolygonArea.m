% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function area = CalculatePolygonArea(V)
len = length(V.x) - 1;
area = 0;
X = V.x;
Y = V.y;
for ii = 1 : len
    v1x = X(ii);
    v1y = Y(ii);
    v2x = X(ii+1);
    v2y = Y(ii+1);
    area = area + v1x * v2y - v1y * v2x;
end
area = 0.5 * abs(area);
end