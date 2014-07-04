%% Trajectory Tracking for Underactuated Vehicles
%
%   For more info see:
%
%   Alessandretti, A., Aguiar, A. P., Jones, C. N.. 
%   Trajectory-tracking and path-following controllers for constrained 
%   underactuated vehicles using Model Predictive Control. 
%   In Control Conference (ECC), 2013 European (pp. 1371?1376).
%

clc; close all; clear all;

v = UAV('InitialConditions',[-3*ones(3,1);reshape(eye(3),9,1)]);

v.controller = TrackingControllerECC14(...
    'Vehicle' , v,... 
    'Epsilon' , -[1;0;1],... % Ultimate tracking error
    'pd'      , @(t)10*[cos(0.05*t);sin(0.05*t);0.1*t],...   % Desired path ...
    'dotPd'   , @(t)10*[-0.05*sin(0.05*t);0.05*cos(0.05*t);0.1],...% ... and its derivative
    'Ke'      , 0.1*eye(3)... % Lyapunov decrease \dot{V} = - e*'Ke*e
    );


h = figure; 
view(3);

va = VirtualArena(v,...
    'StoppingCriteria'    , @(i,aList)i>2000,...
    'StepPlotFunction'    , @stepPlotFunctionPos,...
    'DiscretizationStep'  , 0.1,...
    'HandlePostFirstPlot' , h,...
    'PlottingFrequency'   , 20 ...
    );

ret = va.run();