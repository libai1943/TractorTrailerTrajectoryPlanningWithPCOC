% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function [x, y, theta1, theta2, theta3, theta4] = SimulateForward(node, v, phy)
global vehicle_geometrics_

x0 = node(1);
y0 = node(2);
theta1_0 = node(3);

Nfe = 20;
x = zeros(1,Nfe);
y = zeros(1,Nfe);
theta1 = zeros(1,Nfe);

x(1) = x0;
y(1) = y0;
theta1(1) = theta1_0;

dt = 1 / Nfe;
for ii = 2 : Nfe
    theta1(ii) = dt * tan(phy) * v / vehicle_geometrics_.L_tractor_wheelbase + theta1(ii-1);
    x(ii) = x(ii-1) + dt * cos(theta1(ii)) * v;
    y(ii) = y(ii-1) + dt * sin(theta1(ii)) * v;
end
[x, y, theta1, theta2, theta3, theta4] = Fulfill6DimFrom3Dim(x, y, theta1, node(4), node(5), node(6));