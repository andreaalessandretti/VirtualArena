%% Example dynamic controller
clc; close all; clear all;

p  = 1; % Parameter unknown by the controller
sys = ICtSystem('StateEquation', @(t,x,u,varargin)  p + u, 'nx',1,'nu',1);
sys.initialCondition = 10;

sys.controller = ex07AdaptiveController();
sys.controller.initialCondition = [10;0];

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,sysList)t>30,...
    'DiscretizationStep', 0.1,...
    'PlottingStep'      , 1 ...
    );

log = va.run();

figure
subplot(2,1,1)
plot(log{1}.time,log{1}.stateTrajectory); hold on 
title('State estimation')

plot(log{1}.time,log{1}.controllerStateTrajectory(1,:),'--');hold on 
grid on 

subplot(2,1,2)

plot([log{1}.time(1),log{1}.time(end)],[p,p]);hold on 
title('Parameter estimation')
plot(log{1}.time,log{1}.controllerStateTrajectory(2,:),'--');hold on 
grid on 
