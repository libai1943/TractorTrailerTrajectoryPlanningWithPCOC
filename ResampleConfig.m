% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function [xf, yf, thetaf] = ResampleConfig(x, y, theta)
global IPOPT_
[Nv, Nfe] = size(x);
for ii = 1 : Nv
    for jj = 2 : Nfe
        while (theta(ii,jj) - theta(ii,jj-1) > pi + 0.001)
            theta(ii,jj) = theta(ii,jj) - 2 * pi;
        end
        while (theta(ii,jj) - theta(ii,jj-1) < -pi - 0.001)
            theta(ii,jj) = theta(ii,jj) + 2 * pi;
        end
    end
end

xf = [];
yf = [];
thetaf = [];

for ii = 1 : Nv
    temp_x = [];
    temp_y = [];
    temp_theta = [];
    for jj = 1 : (Nfe-1)
        distance = hypot(x(ii,jj+1)-x(ii,jj),y(ii,jj+1)-y(ii,jj));
        LARGE_NUM = round(distance * 100);
        
        temp = linspace(x(ii,jj), x(ii,jj+1), LARGE_NUM);
        temp = temp(1,1:(LARGE_NUM - 1));
        temp_x = [temp_x, temp];
        temp = linspace(y(ii,jj), y(ii,jj+1), LARGE_NUM);
        temp = temp(1,1:(LARGE_NUM - 1));
        temp_y = [temp_y, temp];
        temp = linspace(theta(ii,jj), theta(ii,jj+1), LARGE_NUM);
        temp = temp(1,1:(LARGE_NUM - 1));
        temp_theta = [temp_theta, temp];
    end
    temp_x = [temp_x, x(ii,Nfe)];
    temp_y = [temp_y, y(ii,Nfe)];
    temp_theta = [temp_theta, theta(ii,Nfe)];
    Len = length(temp_x);
    index = round(linspace(1, Len, IPOPT_.nfe));
    temp_x = temp_x(index);
    temp_y = temp_y(index);
    temp_theta = temp_theta(index);
    
    xf = [xf; temp_x];
    yf = [yf; temp_y];
    thetaf = [thetaf; temp_theta];
end