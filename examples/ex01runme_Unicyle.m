%% Note:
% - Define controller in a separate file
% - Custom StagePlotFunction
% - Custom StoppingCriteria

clc; close all; clear all;

%% Unicycle Model
sys = CtSystem(...
    'StateEquation', @(t,x,u) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)],...
    'nx',3,'nu',2 ...
);

sys.initialCondition = [1;1;0];

desiredPosition = [0;0];

sys.controller  = UniGoToPoint(desiredPosition);

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,sysList)norm(sysList{1}.x(1:2)-desiredPosition)<0.1,...
    'DiscretizationStep', 0.1,...
    'StepPlotFunction'  , @ex01StepPlotFunction ...
    );

log = va.run();

log{1}