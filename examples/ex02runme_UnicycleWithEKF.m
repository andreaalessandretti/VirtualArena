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

sys.controller       = UniGoToPoint(desiredPosition);

%% <<< BEGIN difference from ex01   

dtSys   = DtSystem(sys,dt);

sys.stateObserver = EkfFilter(dtSys,...
                 'StateNoiseMatrix'  , diag(([0.1,0.1,pi/4])/3)^2,...
                 'OutputNoiseMatrix' , diag(([0.1,0.1])/3)^2,...
                 'InitialCondition'  , [[1;-1;0];                  %xHat(0)
                                        10*reshape(eye(3),9,1)]);  %P(0)
                                    
%% <<< END difference from ex01  


sys.initialCondition = [1;1;0];

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,sysList)norm(sysList{1}.x(1:2)-desiredPosition)<0.1,...
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1, ... 
    'StepPlotFunction'  , @ex02StepPlotFunction ...  %% <<< difference from ex01 ( plot estimate )
    );

log = va.run();

log{1}