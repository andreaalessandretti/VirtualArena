%% Nonlinear Model Predictive Control 
clc; close all; clear all

dt = 0.1;

v1 = Unicycle('InitialCondition',[4;4;0]);

v1.useSymbolicEvaluation(); % see help GeneralSystem.useSymbolicEvaluation

op = CtMpcOp( ...
    'System'               , v1,...
    'HorizonLength'        , 0.5,...
    'StageConstraints'     , BoxSet( -[1;pi/4],4:5,[1;pi/4],4:5,5),... % on the variable z=[x;u];
    'StageCost'            , @(t,x,u) x(1:2)'*x(1:2) + u'*u,...
    'TerminalCost'         , @(t,x) 10*x(1:2)'*x(1:2),...
    'TerminalConstraints'  , BoxSet( -100*ones(3,1),1:3,100*ones(3,1),1:3,3)... % on the variable x(T);
    );


mode = 2;
switch mode
    case 1
        
        op = DtMpcOp(op,dt);
        solver = FminconMpcOpSolver('MpcOp',op);
        mpcOpSolOps = {'SolverOptions',optimset('Algorithm','sqp','MaxIter',5)};
        
    case 2
        %% Acado toolkit reuired ( see sourceforge.net/projects/acado/)
        solver = AcadoMpcOpSolver('StepSize',dt,'MpcOp',op);
        mpcOpSolOps = {};
        
end


controller = MpcController(...
    'MpcOp'                 , op,...
    'MpcOpSolver'           , solver,...
    'MpcOpSolverParameters' , mpcOpSolOps, ... 
    'WarmStartMode'         , 1 ...
    );

v1.controller = controller;

a = VirtualArena(v1,...
    'StoppingCriteria'   , @(i,agentsList)norm(agentsList{1}.x(1:2))<2,...
    'StepPlotFunction'   , @stepPlotFunctionMpc, ...
    'DiscretizationStep' , dt);

a.run();

%% Plots
nsteps = 20;

figure

subplot(2,1,1)
hist(v1.controller.log.computationTime(1:nsteps-1));
title('Computation time histogram');
ylabel('Frequency')
xlabel('Computation time')

subplot(2,1,2)
bar(1:nsteps-1,v1.controller.log.computationTime(1:nsteps-1));
title('Computation time');
xlabel('Time step')
ylabel('Computation time')
mean(v1.controller.log.computationTime(1:nsteps-1))