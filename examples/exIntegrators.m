

clc;clear all;close all;

v1 = UAV(...
    'Controller',InlineController(@(x,t)[0.2;0;0;1]),...
    'InitialConditions',[1;1;0;reshape(eye(3),9,1)] ...
    );

dt = 0.1;

a = VirtualArena(v1,...
    'StoppingCriteria',@(i,agentsList)i>20/dt,...
    'StepPlotFunction',@stepPlotFunctionPos, ...
    'PlottingFrequency',10,...
    'DiscretizationStep',dt... %<= add  ",'Integrator',EulerForward()" to the difference
    ); 

a.run();