% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function is_valid = IsTrailerStateValid(x1, y1, theta1)
is_valid = 1;
global vehicle_geometrics_
AX = x1 + vehicle_geometrics_.L_trailer_front_hang * cos(theta1) - vehicle_geometrics_.LHW * sin(theta1);
AY = y1 + vehicle_geometrics_.L_trailer_front_hang * sin(theta1) + vehicle_geometrics_.LHW * cos(theta1);
BX = x1 + vehicle_geometrics_.L_trailer_front_hang * cos(theta1) + vehicle_geometrics_.LHW * sin(theta1);
BY = y1 + vehicle_geometrics_.L_trailer_front_hang * sin(theta1) - vehicle_geometrics_.LHW * cos(theta1);
CX = x1 - vehicle_geometrics_.L_trailer_rear_hang * cos(theta1) + vehicle_geometrics_.LHW * sin(theta1);
CY = y1 - vehicle_geometrics_.L_trailer_rear_hang * sin(theta1) - vehicle_geometrics_.LHW * cos(theta1);
DX = x1 - vehicle_geometrics_.L_trailer_rear_hang * cos(theta1) - vehicle_geometrics_.LHW * sin(theta1);
DY = y1 - vehicle_geometrics_.L_trailer_rear_hang * sin(theta1) + vehicle_geometrics_.LHW * cos(theta1);

PX = [AX,BX,CX,DX];
PY = [AY,BY,CY,DY];

global planning_scale_
a = find(PX > planning_scale_.xmax);
b = find(PX < planning_scale_.xmin);
c = find(PY > planning_scale_.ymax);
d = find(PY < planning_scale_.ymin);

if (length(a) + length(b) + length(c) + length(d) > 0)
    is_valid = 0;
    return;
end

global hybrid_astar_
resolution_x = hybrid_astar_.resolution_x;
resolution_y = hybrid_astar_.resolution_y;
xmin = planning_scale_.xmin;
ymin = planning_scale_.ymin;

PX = [AX,BX,CX,DX];
PY = [AY,BY,CY,DY];
global costmap_
for ii = 1 : length(PX)
    indx = round((PX(ii) - xmin) /  resolution_x) + 1;
    indy = round((PY(ii) - ymin) /  resolution_y) + 1;
    if (costmap_(indx,indy) == 1)
        is_valid = 0;
        return;
    end
end
end