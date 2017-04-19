%% Using a Remote System
%Before running this file run in the terminal the python system:
%python ex04RealSystem.py 

clc; close all; clear all;

dt = 0.2; %%<<< keep 0.2 or update the python file

%% Unicycle Model
sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)],...
    'OutputEquation', @(t,x,varargin) x(1:2), 'ny', 2,...
    'nx',3,'nu',2 ...
);

% System with state and input noise
Q = diag(([0.1,0.1,pi/4])/3)^2;
R = diag(([0.1,0.1])/3)^2;
 
%% <<< BEGIN difference from ex03                                  
realSystem = ex04RemoteUnicycle(...
            'RemoteIp', '127.0.0.1', 'RemotePort', 20001,...
            'LocalIp' , '127.0.0.1', 'LocalPort' , 20002);
 
% Iinitial conditions for the observer
x0Filter= [[15;15;-pi/2] + 0.5*randn(3,1);  %xHat(0)
           10*reshape(eye(3),9,1)];         %P(0)

%% <<< END difference from ex03 

realSystem.stateObserver = EkfFilter(DiscretizedSystem(sys,dt),...
                 'StateNoiseMatrix'  , dt*Q,...
                 'OutputNoiseMatrix' , R,...
                 'InitialCondition'  , x0Filter);

auxiliaryControlLaw =  TrackingController_ECC13(... % <<< attached to the realSystem
    @(t) 10*[sin(0.1*t); cos(0.1*t)] , ... % c
    @(t)    [cos(0.1*t);-sin(0.1*t)] , ... % cDot
    eye(2)                           , ... % K
    [1;0] );

e = @(t,x)auxiliaryControlLaw.computeError(t,x);

mpcOp = ICtMpcOp( ...
    'System'               , sys,...
    'HorizonLength'        , 2*dt,...
    'StageConstraints'     , BoxSet( -[1.1;1.1],4:5,[1.1;1.1],4:5,5),... % on the variable z=[x;u];
    'StageCost'            , @(t,x,u,varargin) e(t,x)'* e(t,x),...
    'TerminalCost'         , @(t,x,varargin) 0.3333*(e(t,x)'* e(t,x))^(3/2)...
    );

dtMpcOp = DiscretizedMpcOp(mpcOp,dt);

realSystem.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
    );

va = VirtualArena(realSystem,...%% <<< difference from ex03
    'StoppingCriteria'  , @(t,sysList)t>70/dt,...
    'ExtraLogs'         , {MeasurementsLog(sys.ny)},... %% <<< difference from ex03
    'StepPlotFunction'  , @ex04StepPlotFunction ...     %% <<< like ex03 withouht state plotting
    );

log = va.run();

hold on 

plot(log{1}.measurements(1,:),log{1}.measurements(2,:),'o')

