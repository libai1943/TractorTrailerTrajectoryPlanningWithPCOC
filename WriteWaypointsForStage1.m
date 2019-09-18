% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function WriteWaypointsForStage1(x,y)
[~, Nfe] = size(x);
delete('Waypoints');
fid = fopen('Waypoints', 'w');
for ii = 1 : Nfe
    fprintf(fid, '%g  1  %f\r\n', ii-1, x(1,ii));
    fprintf(fid, '%g  2  %f\r\n', ii-1, y(1,ii));
end
fclose(fid);