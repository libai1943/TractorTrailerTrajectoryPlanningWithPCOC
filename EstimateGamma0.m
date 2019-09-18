% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function gamma = EstimateGamma0()
Nv = 4;
load AX.txt
Nfe = round(length(AX) / Nv);
AX = reshape(AX, Nfe, Nv)';
load BX.txt
BX = reshape(BX, Nfe, Nv)';
load CX.txt
CX = reshape(CX, Nfe, Nv)';
load DX.txt
DX = reshape(DX, Nfe, Nv)';
load AY.txt
AY = reshape(AY, Nfe, Nv)';
load BY.txt
BY = reshape(BY, Nfe, Nv)';
load CY.txt
CY = reshape(CY, Nfe, Nv)';
load DY.txt
DY = reshape(DY, Nfe, Nv)';

ub = 1;
if (~IsCurrentGammaValid(AX, BX, CX, DX, AY, BY, CY, DY, 0.1))
    if (~IsCurrentGammaValid(AX, BX, CX, DX, AY, BY, CY, DY, 0.001))
        error 'Failed to derive a valid gamma_0 at the very beginning of Stage 3';
    end
end
lb = 0.1;
threshold = 0.01;
is_reduction_ready = 0;

% Binary search
while (~is_reduction_ready)
    trial = 0.5 * (ub + lb);
    if (IsCurrentGammaValid(AX, BX, CX, DX, AY, BY, CY, DY, trial))
        lb = trial;
    else
        ub = trial;
    end
    if (ub - lb < threshold)
        is_reduction_ready = 1;
    end
end
gamma = lb;
end