%% Note:
% - System with output
% - Automatic Discretization
% - Automatic computation of linearized model
% - Automatic generation of an Extended Kalman Filter

clc;close all;clear all;

dt = 0.1;

%% Unicycle Model
sys = CtSystem(...
    'StateEquation', @(t,x,u) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)],...
    'OutputEquation', @(t,x) x(1:2), 'ny', 2,... %% <<< difference from ex01 ( e.g., GPS )
    'nx',3,'nu',2 ...
);

desiredPosition      = [0;0];

%% <<< BEGIN difference from ex01   

realSystem = CtSystem(...
    'StateEquation', @(t,x,u) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)] + [0.1*randn(2,1);randn(1,1)*pi/8],...
    'OutputEquation', @(t,x) x(1:2), 'ny', 2,...
    'nx',3,'nu',2 ...
);

realSystem.stateObserver = EkfFilter(DtSystem(sys,dt),...
                 'StateNoiseMatrix'  , diag(([0.1,0.1,pi/4])/3)^2,...
                 'OutputNoiseMatrix' , diag(([0.1,0.1])/3)^2,...
                 'InitialCondition'  , repmat({[[1;-1;0];                  %xHat(0)
                                        10*reshape(eye(3),9,1)]},1,10*4));  %P(0)

realSystem.initialCondition = repmat({[1;1;pi/2],-[1;1;-pi/2],[1;-1;-pi/2],[-1;1;-pi/2]},1,10);
                                    
realSystem.controller = UniGoToPoint(desiredPosition);



%% <<< END difference from ex01  

va = VirtualArena(realSystem,...
    'StoppingCriteria'  , @(t,sysList)norm(sysList{1}.x(1:2)-desiredPosition)<0.01,...
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1, ... 
    'StepPlotFunction'  , @ex02StepPlotFunction ...  %% <<< difference from ex01 ( plot estimate )
    );

log = va.run();