%% exOffLineFilterTuning
% The tuning of the state observer is a crucial step in the design of
% high-performing output feedback control schemes.
%
% In this tutorial we illustrate how to use VirtualArena to tune a filter
% using measurements from an experiment or, in this specific example, from a
% previous simulation.

clc;clear all; close all;

% System discretization
dt  = 0.05;


%% Simulation Noisy Vehicle / Experimental Results

% System Model

vehicleModel = Unicycle('OutputEquation', @(t,x) x(1:2), 'ny',2 );         

% Real System Model
% In this case we generate measurament simulating a noisy system. In the
% real case, one would take the measurements form the Real System

Qsys = 0.5*eye(3);
Rsys = 5*eye(2);

vehicleReal = CtSystem(...
    'StateEquation'    , @(t,x,u) vehicleModel.f(t,x,u) + chol(Qsys)*randn(vehicleModel.nx,1),...
    'InitialCondition', randn(3,1),...
    'OutputEquation'   , @(t,x) x(1:2) + chol(Rsys)*randn(vehicleModel.ny,1), 'ny',2 ... % Add an observation model
    );  

% Extended Bucy-Kalman Filter
                                                        
vehicleReal.stateObserver = EkbfFilter(vehicleModel,...
            'StateNoiseMatrix'  , Qsys,...
            'OutputNoiseMatrix' , Rsys,...
            'InitialCondition' , [2*ones(3,1);
                                   10*reshape(eye(3),9,1)]);
        

vehicleReal.controller = UniGoToPoint([50;50]);

a = VirtualArena(vehicleReal,...
    'ExtraLogs'        ,   MeasurementsLog(vehicleModel.ny), ...
    'StoppingCriteria'   , @(i,agentsList)i>15,...
    'StepPlotFunction'   , @stepPlotFunctionEkf, ...
    'PlottingStep'       , 1,...
    'DiscretizationStep' , dt...
    );

logs = a.run();


%% Simulation Replaying the measurements and control inputs from the experiment

replaySys = ReplayMeasuramentsCtSystem('TimeLog',logs{1}.time,'MeasurementLog',logs{1}.measurements,'nu',2);
replaySys.controller = ReplayController(logs{1}.time,logs{1}.inputTrajectory);

% At this point, it is possible to use the measurements and control inputs 
% from the experiment to improve the observer (e.g., testing a different
% models or parameters).

replaySys.stateObserver = EkbfFilter(vehicleModel,...
            'StateNoiseMatrix'  , 10*Qsys,... %noise injection
            'OutputNoiseMatrix' , 10*Rsys,... %noise injection
            'InitialCondition' , [2*ones(3,1);
                                   10*reshape(eye(3),9,1)]);
        
                               
a2 = VirtualArena(replaySys,...
    'ExtraLogs'        ,   MeasurementsLog(vehicleModel.ny), ...
    'StoppingCriteria'   , @(i,agentsList)i>15,...
    'StepPlotFunction'   , @stepPlotFunctionEkf, ...
    'PlottingStep'       , 1,...
    'DiscretizationStep' , dt...
    );

logs2 = a2.run();

                               
                               