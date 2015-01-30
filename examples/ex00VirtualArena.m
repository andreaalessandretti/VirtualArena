%% Simulate same systems for different initial conditions
clc; clear all; close all;

mode = 1;
%0: Simulate from a single initial condition
%1: Simulate from multiple initial conditions
switch mode
    case 0
        init1 = [10;0;0];
        init2 = [0;10;0];
        initc = [0;0];
        
    case 1 
        init1 = {zeros(3,1),[10;0;0]};
        init2 = {10*ones(3,1),[0;10;0]};  
        initc = {[0;0],[0;0]};
end
        

v1 = Unicycle(...
    'Controller',UniGoToPoint([5;5]),...
    'InitialConditions',init1);

v2 = Unicycle(...
    'Controller',UniGoToPointPid([5;5],...
        'InitialConditions',initc,...
        'PidGains',[1,0.1,1]),...
    'InitialConditions', init2);

a = VirtualArena({v1,v2},...
    'StoppingCriteria',@(i,as)i>10,...
    'StepPlotFunction',@stepPlotFunctionPos, ...
    'DiscretizationStep',0.1);

a.run();
