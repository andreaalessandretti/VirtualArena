% The full description of the control algorithm can be found in the paper
%
% Alessandretti A., Aguiar A. P., Jones C. N. 
% Trajectory-tracking and path-following controllers for constrained underactuated vehicles using Model Predictive Control. 
% In Proc. of the 2013 European Control Conference (pp. 1371?1376).
%
% http://wiki.epfl.ch/projects/mpcttpf

addpath('./lib');clc; close all;clear all

dt = 0.1;

%% Vehicle
v = Unicycle('InitialConditions',[8;-5;0]);

%% Constraints
% Imposed
uBound    = 3;    % |u| < uBound
rBound    = 2*pi; % |r| < rBound
% Observed
dPdBound  = 0.5;    % | d/dt p_d(t) | <= dPdBound, with p_d(t) desired position


%% Mpc Optimization Problem (MpcOp) for Trajectory Tracking
mpcOp = MpcOpTrackingECC13(...
    'System'           , v,...
    'Epsilon'          , -[0.2;0],... % Ultimate tracking offset
    'pd'               , @(t)10*[cos(0.05*t);sin(0.05*t)],...   % Desired path ...
    'dotPd'            , @(t)10*[-0.05*sin(0.05*t);0.05*cos(0.05*t)],...% ... and its derivative
    'Ke'               , 0.1*eye(2),... % Lyapunov decrease \dot{V} = - e*'Ke*e
    'HorizonLength'    , 0.5,...
    'StageConstraints' , BoxSet([-uBound;-rBound],4:5,[uBound;rBound],4:5,5),... % on the variable z=[x;u];
    'O'                , eye(2),...
    'Q'                , 10*eye(2)...
    );

mpcOp.addTerminalErrorConstraint(dPdBound);


%% MpcOp solver
if exist('BEGIN_ACADO') % Use ACADO if available, else use fmincon
    
    mpcOpSolver = AcadoMpcOpSolver(...
        'MpcOp'                            , mpcOp,...
        'StepSize'                         , dt,...
        'AcadoOptimizationAlgorithmOptions', {'KKT_TOLERANCE',1e-4,'MAX_NUM_ITERATIONS',30 } ...
        );
    solverParameters = {};
    
else
    
    disp('WARNING: Modify solverParameters to improve the quality of the solution.')
    
    mpcOp.useSymbolicEvaluation();
    mpcOpSolver      = FminconMpcOpSolver('MpcOp', DtMpcOp(mpcOp,dt),'DiscretizationStep',dt);
    solverParameters = {'SolverOptions',optimset('Algorithm','interior-point','MaxIter',5)};
    
end

v.controller = MpcController(...
    'MpcOp'                 , mpcOp,...
    'MpcOpSolver'           , mpcOpSolver,...
    'WarmStartMode'         , 1, ...
    'MpcOpSolverParameters' , solverParameters ...
    );

a = VirtualArena(v,...
    'StoppingCriteria'   , @(i,agentsList) i>200,...
    'StepPlotFunction'   , @(systemsList,log,oldHandles,k) stepPlotFunctionMpc(systemsList,log,oldHandles,k,dt,1), ...
    'InitPlotFunction'   , @initPlotFunctionMpc,...
    ...%'VideoName', 'tracking', ...
    'DiscretizationStep' , dt);

ret = a.run();
