% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function [V1, V2, V3, V4] = ConvertVariablesToPolygons(AX, BX, CX, DX, AY, BY, CY, DY)
V1.x = [AX(1), BX(1), CX(1), DX(1), AX(1)];
V1.y = [AY(1), BY(1), CY(1), DY(1), AY(1)];
V1.A = CalculatePolygonArea(V1);

V2.x = [AX(2), BX(2), CX(2), DX(2), AX(2)];
V2.y = [AY(2), BY(2), CY(2), DY(2), AY(2)];
V2.A = CalculatePolygonArea(V2);

V3.x = [AX(3), BX(3), CX(3), DX(3), AX(3)];
V3.y = [AY(3), BY(3), CY(3), DY(3), AY(3)];
V3.A = CalculatePolygonArea(V3);

V4.x = [AX(4), BX(4), CX(4), DX(4), AX(4)];
V4.y = [AY(4), BY(4), CY(4), DY(4), AY(4)];
V4.A = CalculatePolygonArea(V4);
end