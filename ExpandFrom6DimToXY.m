% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function [x2, y2, x3, y3, x4, y4] = ExpandFrom6DimToXY(x1, y1, theta1, theta2, theta3, theta4)
global vehicle_geometrics_
Nfe = length(x1);
x2 = zeros(1,Nfe);
y2 = x2;
x3 = x2;
y3 = x2;
x4 = x2;
y4 = x2;
L = vehicle_geometrics_.L;
M = vehicle_geometrics_.M;

x2 = x1 - L(2) * cos(theta2) - M(1) * cos(theta1);
y2 = y1 - L(2) * sin(theta2) - M(1) * sin(theta1);
x3 = x2 - L(3) * cos(theta3) - M(2) * cos(theta2);
y3 = y2 - L(3) * sin(theta3) - M(2) * sin(theta2);
x4 = x3 - L(4) * cos(theta4) - M(3) * cos(theta3);
y4 = y3 - L(4) * sin(theta4) - M(3) * sin(theta3);