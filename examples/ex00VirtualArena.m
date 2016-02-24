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
    'InitialCondition',init1);

v2 = Unicycle(...
    'Controller',UniGoToPointPid([5;5],...
        'InitialCondition',initc,...
        'PidGains',[1,0.1,1]),...
    'InitialCondition', init2);

va = VirtualArena({v1,v2},...
    'StoppingCriteria',@(t,as)t>10,...
    'StepPlotFunction',@stepPlotFunctionPos, ...
    'DisplaySelector', DisplayVerbose(),... 
    'DiscretizationStep',0.1);

ret = va.run();
