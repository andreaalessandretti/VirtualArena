clc; close all; clear all;

dt = 0.01;

%% Unicycle Model
sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)],...
    'nx',3,'nu',2 ...
    );

cdes    = @(t) 10*[sin(0.1*t); cos(0.1*t)] ;
cdesDot = @(t)    [cos(0.1*t);-sin(0.1*t)] ;
K     = eye(2);
epsilon = [1;0];

Delta = [[1;0],[-epsilon(2);epsilon(1)]];
R = @(x) [cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
p = @(x)x(1:2);
e = @(t,x)R(x)'*(p(x)-cdes(t))+epsilon;

sys.controller = IController(@(t,x)-Delta\( K*e(t,x) -R(x)'*cdesDot(t)));

sys.initialCondition = {[15;15;-pi/2],-[15;15;-pi/2],[15;-15;pi],[-15;15;-pi/2]};

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,sysList)t>70,...
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1, ...
    'StepPlotFunction'  , @ex01StepPlotFunction ...
    );

log = va.run();
