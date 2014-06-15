clc;close all;clear all;

v = UAV();
v.initialConditions =[-3*ones(3,1);reshape(eye(3),9,1)];

k = 0.05;
v.controller = TrackingControllerECC14(...
    'Vehicle' , v,... % Vehicle
    'Epsilon' , -[1;0;1],... %epsilon
    'pd'      , @(t)10*[cos(k*t);sin(k*t);0.1*t],...
    'dotPd'   , @(t)10*[-k*sin(k*t);k*cos(k*t);0.1],...%dDp(t)
    'Ke'      , 0.1*eye(3)...%Ke
    );

h = figure; 
view(3);
grid on

va = VirtualArena(v,...
    'StoppingCriteria',@(i,aList)i>2000,...
    'StepPlotFunction',@stepPlotFunctionPos,...
    'DiscretizationStep',0.1,...
    'HandlePostFirstPlot',h,...
    'PlottingFrequency',20 ...
    );

ret = va.run();