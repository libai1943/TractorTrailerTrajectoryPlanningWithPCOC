% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function [x, y, theta1, theta2, theta3, theta4] = Fulfill6DimFrom3Dim(x, y, theta1, theta2_0, theta3_0, theta4_0)
Nfe = length(x);
theta2 = zeros(1,Nfe); theta2(1,1) = theta2_0;
theta3 = theta2; theta3(1,1) = theta3_0;
theta4 = theta2; theta4(1,1) = theta4_0;
global vehicle_geometrics_
M = vehicle_geometrics_.M;
L = vehicle_geometrics_.L;

for ii = 2 : Nfe
    % specify v1
    v1_candidate1 = (x(ii) - x(ii-1)) / cos(theta1(ii));
    v1_candidate2 = (y(ii) - y(ii-1)) / sin(theta1(ii));
    if (abs(v1_candidate1) > abs(v1_candidate2))
        v1 = v1_candidate2;
    else
        v1 = v1_candidate1;
    end
    % specify theta2
    theta2(ii) = theta2(ii-1) + (v1 * sin(theta1(ii-1) - theta2(ii-1)) - M(1) * cos(theta1(ii-1) - theta2(ii-1)) * (theta1(ii) - theta1(ii-1))) / L(2);
    % specify v2
    v2 = v1 * cos(theta1(ii) - theta2(ii)) + M(1) * sin(theta1(ii) - theta2(ii)) * (theta1(ii) - theta1(ii-1));
    % specify theta3
    theta3(ii) = theta3(ii-1) + (v2 * sin(theta2(ii-1) - theta3(ii-1)) - M(2) * cos(theta2(ii-1) - theta3(ii-1)) * (theta2(ii) - theta2(ii-1))) / L(3);
    % specify v3
    v3 = v2 * cos(theta2(ii) - theta3(ii)) + M(2) * sin(theta2(ii) - theta3(ii)) * (theta2(ii) - theta2(ii-1));
    % specify theta4
    theta4(ii) = theta4(ii-1) + (v3 * sin(theta3(ii-1) - theta4(ii-1)) - M(3) * cos(theta3(ii-1) - theta4(ii-1)) * (theta3(ii) - theta3(ii-1))) / L(4);
end
