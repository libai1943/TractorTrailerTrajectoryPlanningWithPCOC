% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function xx = ResampleConfigSimple(x,NE)
[Nv, Nfe] = size(x);
xx = [];
LARGE_NUM = 100;
for ii = 1 : Nv
    temp_x = [];
    for jj = 1 : (Nfe-1)
        temp = linspace(x(ii,jj), x(ii,jj+1), LARGE_NUM);
        temp_x = [temp_x, temp(1,1:(LARGE_NUM - 1))];
    end
    temp_x = [temp_x, x(ii,Nfe)];
    index = round(linspace(1,length(temp_x),NE));
    xx = [xx; temp_x(index)];
end