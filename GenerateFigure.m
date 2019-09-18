% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function GenerateFigure(phy, x, y, theta, AX, BX, CX, DX, AY, BY, CY, DY)
global vehicle_geometrics_
global vehicle_boundary_value_configs_
final_xc = vehicle_boundary_value_configs_.xc_end;
final_yc = vehicle_boundary_value_configs_.yc_end;
final_x_half_length = vehicle_boundary_value_configs_.x_end_half_width;
final_y_half_length = vehicle_boundary_value_configs_.y_end_half_width;
final_A = [final_xc-final_x_half_length, final_yc+final_y_half_length];
final_B = [final_xc+final_x_half_length, final_yc+final_y_half_length];
final_C = [final_xc+final_x_half_length, final_yc-final_y_half_length];
final_D = [final_xc-final_x_half_length, final_yc-final_y_half_length];
final_PP = [final_A(1),final_B(1),final_C(1),final_D(1),final_A(1); final_A(2),final_B(2),final_C(2),final_D(2),final_A(2)];

Nv = 4;
global colorpool
colorpool = [237,28,36; 0,162,232; 34,177,76; 255,127,39]./255;
number_of_frame = 120;

x = ResampleConfigSimple(x,number_of_frame);
y = ResampleConfigSimple(y,number_of_frame);
theta = ResampleConfigSimple(theta,number_of_frame);
AX = ResampleConfigSimple(AX,number_of_frame);
BX = ResampleConfigSimple(BX,number_of_frame);
CX = ResampleConfigSimple(CX,number_of_frame);
DX = ResampleConfigSimple(DX,number_of_frame);
AY = ResampleConfigSimple(AY,number_of_frame);
BY = ResampleConfigSimple(BY,number_of_frame);
CY = ResampleConfigSimple(CY,number_of_frame);
DY = ResampleConfigSimple(DY,number_of_frame);

for ind = 1 : number_of_frame
    set(0,'DefaultLineLineWidth',0.5);
    for ii = 1 : Nv
        xc = x(ii,ind);
        yc = y(ii,ind);
        tc = theta(ii,ind);
        A = [AX(ii,ind), AY(ii,ind)];
        C = [CX(ii,ind), CY(ii,ind)];
        B = [BX(ii,ind), BY(ii,ind)];
        D = [DX(ii,ind), DY(ii,ind)];
        P = [A(1),B(1),C(1),D(1),A(1);A(2),B(2),C(2),D(2),A(2)];
        plot(P(1,:),P(2,:),'Color',colorpool(ii,:));
    end
    % set(0,'DefaultLineLineWidth',2);
end
set(0,'DefaultLineLineWidth',1.5);
plot(final_PP(1,:),final_PP(2,:),'k--');
% for ii = 1 : Nv
%     plot(x(ii, :), y(ii, :), 'k');
% end