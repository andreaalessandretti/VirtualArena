clc; close all; clear all;

dt = 0.1;

%% Unicycle Model
sys = ICtSystem(...
    'StateEquation', @(t,x,u) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)],...
    'nx',3,'nu',2 ...
);

desiredPosition      = [0;0];

sys.controller       = UniGoToPoint(desiredPosition);

sys.initialCondition = [1;1;0];

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,sysList)norm(sysList{1}.x(1:2)-desiredPosition)<0.1,...
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1, ...
    'StepPlotFunction'  , @ex01StepPlotFunction ...
    );

log = va.run();
