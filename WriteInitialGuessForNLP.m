% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function WriteInitialGuessForNLP(x, y, theta)
global vehicle_geometrics_
global vehicle_physics_
phy_max = vehicle_physics_.phy_max;
v_max = vehicle_physics_.v_max;
M = vehicle_geometrics_.M;

[Nv, NE] = size(x);
phy = zeros(1, NE);
dt = zeros(1, NE);
v1 = zeros(1, NE);
v2 = zeros(1, NE);
v3 = zeros(1, NE);
v4 = zeros(1, NE);
theta1 = theta(1,:);
theta2 = theta(2,:);
theta3 = theta(3,:);

for ii = 1 : (NE-1)
    % Specify v(t)
    addtion = (x(1,ii+1) - x(1,ii)) * cos(theta(1,ii)) + (y(1,ii+1) - y(1,ii)) * sin(theta(1,ii));
    if (addtion > 0)
        v1(ii) = v_max;
    else
        v1(ii) = -v_max;
    end
    % Specify dt
    ds = hypot(x(1,ii+1) - x(1,ii), y(1,ii+1) - y(1,ii));
    dt(ii) = ds / abs(v1(ii));
    % Specify phy(t)
    dtheta = (theta(ii+1) - theta(ii)) / dt(ii);
    phy(ii) = atan(dtheta * vehicle_geometrics_.L_tractor_wheelbase / v1(ii));
    if (phy(ii) > phy_max)
        phy(ii) = phy_max;
    elseif (phy(ii) < -phy_max)
        phy(ii) = -phy_max;
    end
end

delete('ig0.INIVAL');
fid = fopen('ig0.INIVAL', 'w');
for ii = 0 : (NE-1)
    fprintf(fid, 'fix phy[%g] := %f;\r\n', ii, phy(ii+1));
end

for ii = 2 : NE
    v2 = v1 * cos(theta1(ii) - theta2(ii)) + M(1) * sin(theta1(ii) - theta2(ii)) * (theta1(ii) - theta1(ii-1));
    v3 = v2 * cos(theta2(ii) - theta3(ii)) + M(2) * sin(theta2(ii) - theta3(ii)) * (theta2(ii) - theta2(ii-1));
end
v = [v1; v2; v3; v4];
for ii = 1 : Nv
    for jj = 0 : (NE-1)
        fprintf(fid, 'fix x[%g,%g] := %f;\r\n', jj, ii, x(ii,jj+1));
        fprintf(fid, 'fix y[%g,%g] := %f;\r\n', jj, ii, y(ii,jj+1));
        fprintf(fid, 'fix theta[%g,%g] := %f;\r\n', jj, ii, theta(ii,jj+1));
        fprintf(fid, 'fix v[%g,%g] := %f;\r\n', jj, ii, v(ii,jj+1));
    end
end
fclose(fid);

delete('NE');
fid = fopen('NE', 'w');
fprintf(fid, '%g \r\n', NE-1);
fclose(fid);

delete('TF');
fid = fopen('TF', 'w');
fprintf(fid, '%f \r\n', sum(dt));
fclose(fid);
!ampl r0.run