%% Extended Kalman Filter

clc;clear all;close all;

% System discretization
dt  = 0.1;

v1 = Unicycle(...
    'Controller'       , UniGoToPoint([5;5]),...
    'InitialConditions', zeros(3,1),...
    'OutputEquation'   , @(t,x,w) x + w, 'ny',3,... % Add an observation model
    'Q', 0.1*eye(3), 'R' , 0.1*eye(3));         


observer = 'ebkf';

Qobs = eye(3);
Robs = 20*eye(3);

switch observer
    case 'ekf' %%Extended Kalman Filter
        
        dtV = DtSystem(v1,dt);
        
        %see help GeneralSystem.computeLinearization
        dtV.computeLinearization('Sampled');
        
        obs = EkfFilter(dtV,...
            'StateNoiseMatrix'  , dt*Qobs,...
            'OutputNoiseMatrix' , (1/dt)*Robs,...
            'InitialConditions' , [2*ones(3,1);
                                   10*reshape(eye(3),9,1)]);
        
    case 'ebkf' %% Extended Bucy-Kalman Filter
        
        %see help GeneralSystem.computeLinearization
        v1.computeLinearization('Sampled');
        
        ebkf = EkbfFilter(v1,...
            'StateNoiseMatrix'  , Qobs,...
            'OutputNoiseMatrix' , Robs,...
            'InitialConditions' , [2*ones(3,1);
                                   10*reshape(eye(3),9,1)]);
        obs = ebkf;
end

v1.stateObserver = obs;

a = VirtualArena(v1,...
    'StoppingCriteria'   , @(i,agentsList)i>300,...
    'StepPlotFunction'   , @stepPlotFunctionEkf, ...
    'PlottingFrequency'  , 10,...
    'DiscretizationStep' , dt);

logs = a.run();
