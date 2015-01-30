%% Extended Kalman Filter

clc;clear all;close all;

% System discretization
dt  = 0.05;


Qobs = 0.5*eye(3);
Robs = 5*eye(3);


vehicleModel = Unicycle('OutputEquation', @(t,x) x, 'ny',3 );         

vehicleReal = CtSystem(...
    'StateEquation'    , @(t,x,u) vehicleModel.f(t,x,u) + chol(Qobs)*randn(vehicleModel.nx,1),...
    'InitialConditions', randn(3,1),...
    'OutputEquation'   , @(t,x) x + chol(Robs)*randn(vehicleModel.ny,1), 'ny',3 ... % Add an observation model
    );  



observer = 'ebkf';

switch observer
    case 'ekf' %%Extended Kalman Filter
        
        dtVehicleModel = DtSystem(vehicleModel,dt);
        
        %see help GeneralSystem.computeLinearization
        dtVehicleModel.computeLinearization('Sampled');
        
        obs = EkfFilter(dtVehicleModel,...
            'StateNoiseMatrix'  , dt*Qobs,...
            'OutputNoiseMatrix' , (1/dt)*Robs,...
            'InitialConditions' , [2*ones(3,1);
                                   10*reshape(eye(3),9,1)]);
        
    case 'ebkf' %% Extended Bucy-Kalman Filter
        
        %see help GeneralSystem.computeLinearization
        vehicleModel.computeLinearization('Sampled');
        
        ebkf = EkbfFilter(vehicleModel,...
            'StateNoiseMatrix'  , Qobs,...
            'OutputNoiseMatrix' , Robs,...
            'InitialConditions' , [2*ones(3,1);
                                   10*reshape(eye(3),9,1)]);
        obs = ebkf;
end

vehicleReal.stateObserver = obs;

vehicleReal.controller = UniGoToPoint([50;50]);


a = VirtualArena(vehicleReal,...
    'StoppingCriteria'   , @(i,agentsList)i>15,...
    'StepPlotFunction'   , @stepPlotFunctionEkf, ...
    'PlottingFrequency'  , 10,...
    'DiscretizationStep' , dt...
    );

logs = a.run();
