clc; close all; clear all

dt = 0.1;

v1 = Unicycle('InitialConditions',[4;4;0]);

v1.useSymbolicEvaluation();

nsteps = 20;

op = CtMpcOp( ...
    'System', v1,...
    'HorizonLength', 0.5,...
    'StageConstraints', BoxSet( -[1;pi/4],4:5,[1;pi/4],4:5,5),... % on the vatiable z=[x;u];
    'StageCost', @(x,u) x(1:2)'*x(1:2) + u'*u,...
    'TerminalCost', @(x) 10*x(1:2)'*x(1:2));

solver = FminconMpcOpSolver();

op = DtMpcOp(op,dt);

solver.init(op);

controller = MpcController(...
    'MpcOp',op,...
    'MpcOpSolver',solver,...
    'MpcOpSolverParameters',{'SolverOptions',optimset('Algorithm','sqp','MaxIter',5)}, ... % 
    'WarmStartMode',1 ...
    );

v1.controller = controller;

a = VirtualArena(v1,...
    'StoppingCriteria',@(i,agentsList)norm(agentsList{1}.x(1:2))<2,...
    'StepPlotFunction',@stepPlotFunctionMpc, ...
    'DiscretizationStep',0.05);

a.run();

%% Plots

figure
subplot(2,1,1)
hist(v1.controller.log.computationTime(1:nsteps-1));
title('Computation time histogram');
ylabel('frequency')
xlabel('computation time')

subplot(2,1,2)
bar(1:nsteps-1,v1.controller.log.computationTime(1:nsteps-1));
title('Computation time');
xlabel('time step')
ylabel('computation time')
mean(v1.controller.log.computationTime(1:nsteps-1))