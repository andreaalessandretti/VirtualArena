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
% we add an extra state which is the parameter of the path
v    = Unicycle('InitialCondition',[8;-5;0;0]);
v.nx = v.nx + 1;
v.nu = v.nu + 1;
oldf = v.f;
v.f  = @(t,x,u)[oldf(t,x,u);u(v.nu)];

%% Constraints
% Imposed
uBound      = 3;    % |u| < uBound
rBound      = 2*pi; % |r| < rBound
dGammaBound = 5;    % | d/dt gamma(t) | < dGammaBound


% Observed
dPdBound  = 0.5;    % | d/dt p_d(gamma(t)) | <= dPdBound, with p_d(t)
% desired position and with | d/dt gamma(t) | < dGammaBound

%% Mpc Optimization Problem (MpcOp) for Trajectory Tracking
mpcOp = MpcOpPathFollowingECC13(...
    'System'           , v,...
    'Epsilon'          , -[0.2;0],... % Ultimate tracking offset
    'pd'               , @(t)10*[cos(0.05*t);sin(0.05*t)],...   % Desired path ...
    'dotPd'            , @(t)10*[-0.05*sin(0.05*t);0.05*cos(0.05*t)],...% ... and its derivative
    'Ke'               , 0.1*eye(2),... % Lyapunov decrease \dot{V} = - e*'Ke*e
    'HorizonLength'    , 0.5,...
    'StageConstraints' , BoxSet([-uBound;-rBound;-dGammaBound],5:7,[uBound;rBound;dGammaBound],5:7,7),... % on the variable z=[x;u];
    'O'                , eye(2),...
    'Q'                , 10*eye(2),...
    'o'                , 1,...
    'dGammaDes'        , 1 ...
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


%% Mpc Controller
v.controller = MpcController(...
    'MpcOp'                 , mpcOp,...
    'MpcOpSolver'           , mpcOpSolver,...
    'WarmStartMode'         , 1, ...
    'MpcOpSolverParameters' , solverParameters ...
    );


%% Simulator
a = VirtualArena(v,...
    'StoppingCriteria'   , @(i,agentsList) i>200,...
    'StepPlotFunction'   , @(systemsList,log,oldHandles,k) stepPlotFunctionMpc(systemsList,log,oldHandles,k,dt,0), ...
    'InitPlotFunction'   , @initPlotFunctionMpc,...
    ...'VideoName', 'pathfollowing', ...
    'DiscretizationStep' , dt);

ret = a.run();
