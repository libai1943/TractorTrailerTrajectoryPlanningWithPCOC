% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================

%   Copyright (C) 2019 Bai Li
%   User must cite the following article if they utilize these codes for
%   new publications: Bai Li et al., "Tractor-Trailer Vehicle Trajectory
%   Planning in Narrow Environments with a Progressively Constrained
%   Optimal Control Approach", To appear in IEEE TRANSACTIONS ON
%   INTELLIGENT VEHICLES.

% ==============================================================================
% (Very Important) The AMPL utilized in this pack is just a TRIAL version.
% The users must delete AMPL.exe in this version after trying it, and
% then apply for their own valid license files through following the
% official instructions at https://ampl.com/try-ampl/request-a-full-trial/

% ==============================================================================
% Specify test case. Note that we only support entering one of the six
% values: 1.1, 1.2, 2.1, 2.2, 3.1 and 3.2.
test_case_number = 3.2;
global case_id
case_id = round(test_case_number);
if (mod(test_case_number * 10, 10) == 2)
    off_axle_flag = 1;
else
    off_axle_flag = 0;
end

global obstacle_vertexes_
global costmap_

InitParams;
WriteFilesForNLPSetting();
obstacle_vertexes_ = GenerateObstacles();
Nobs = size(obstacle_vertexes_, 2);
costmap_ = CreateCostmap();

% Stage 1
[x, y, theta] = PlanHybridAStarPath();
[x, y, theta] = ResampleConfig(x, y, theta);
WriteInitialGuessForNLP(x, y, theta);
WriteWaypointsForStage1(x,y);
!ampl r1.run
% Stage 2
!ampl r15.run
!ampl r2.run
if (~GetOptimizationStatus())
    error 'Failed at Stage 2'
end
% Stage 3
gamma0 = EstimateGamma0();
step = 0.05; % Initial step
alpha = 0.8; % Setting of \alpha_reduce
Nexpand = 20; % Threshold of successive successful cycles that would make step increase
epsilon_exit = 1e-5; % Threshold to exit Algorithm 1 with a failure
gamma_achieved = gamma0 - step;
counter = 0;
% Algorithm 1 within Stage 3
while (gamma_achieved ~= 1)
    if ((gamma_achieved + step) <= 1)
        gamma_trial = gamma_achieved + step;
    else
        gamma_trial = 1;
    end
    GenerateObstacleVertexes(gamma_trial);
    !ampl r3.run
    if (GetOptimizationStatus() == 1)
        gamma_achieved = gamma_trial;
        counter = counter + 1;
    else
        step = step .* alpha;
        counter = 0;
    end
    if (counter >= Nexpand)
        step = step ./ alpha;
        counter = 0;
    end
    fclose('all');
    if (step <= epsilon_exit)
        error 'Intermediate subproblem solution failed due to epsilon exit criterion at Stage 3'
    end
end
!ampl r4.run

% Run asd or dsa would have the results plotted. Try it!
dsa