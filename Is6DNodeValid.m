% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function is_collision_free = Is6DNodeValid(x1, y1, theta1, theta2, theta3, theta4)
is_collision_free = 1;
err = abs([theta1 - theta2, theta2 - theta3, theta3 - theta4]);
jackknife_index = find(err > pi / 2);
if (length(jackknife_index) > 0)
    is_collision_free = 0;
    return;
end

if (~IsTractorStateValid(x1, y1, theta1))
    is_collision_free = 0;
    return;
end
[x2, y2, x3, y3, x4, y4] = ExpandFrom6DimToXY(x1, y1, theta1, theta2, theta3, theta4);
if (~IsTrailerStateValid(x2, y2, theta2))
    is_collision_free = 0;
    return;
end

if (~IsTrailerStateValid(x3, y3, theta3))
    is_collision_free = 0;
    return;
end

if (~IsTrailerStateValid(x4, y4, theta4))
    is_collision_free = 0;
    return;
end
end