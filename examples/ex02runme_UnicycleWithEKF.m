clc; close all; clear all;

dt = 0.05;

%% Unicycle Model
sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)],...
    'OutputEquation', @(t,x,varargin) x(1:2), 'ny', 2,... %% <<< difference from ex01 ( e.g., GPS )
    'nx',3,'nu',2 ...
);

%% <<< BEGIN difference from ex01   

% System with state and input noise covariance matrices
Q = diag(([0.1,0.1,pi/4])/3)^2;
R = diag(([0.1,0.1])/3)^2;

realSystem = ICtSystem(...
    'StateEquation',  @(t,x,u,varargin) sys.f(t,x,u,varargin{:}) + chol(Q)*randn(3,1),...
    'OutputEquation', @(t,x,varargin)   sys.h(t,x,varargin{:})   + chol(R)*randn(2,1), 'ny', 2,...
    'nx',3,'nu',2 ...
);

% Initial conditions for the system ...
realSystem.initialCondition = repmat({[15;15;-pi/2],-[15;15;-pi/2],[15;-15;-pi],[-15;15;-pi/2]},1,10); 

% ... and associated initial conditions for the observer
for ii = 1:length(realSystem.initialCondition)
    x0Filter{ii} = [realSystem.initialCondition{ii} + 5*randn(3,1);  %xHat(0)
                    10*reshape(eye(3),9,1)                          ]; %P(0)
end

realSystem.stateObserver = EkfFilter(DiscretizedSystem(sys,dt),...
                 'StateNoiseMatrix'  , dt*Q,...
                 'OutputNoiseMatrix' , R,...
                 'InitialCondition'  , x0Filter);
%% <<< END difference from ex01  

realSystem.controller = TrackingController_ECC13(... % <<< attached to the realSystem
    @(t) 10*[sin(0.1*t); cos(0.1*t)] , ... % c
    @(t)    [cos(0.1*t);-sin(0.1*t)] , ... % cDot
    eye(2)                           , ... % K
    [1;0] );

e = @(t,x)realSystem.controller.computeError(t,x);

va = VirtualArena(realSystem,... % <<< simulate the realSystem
    'StoppingCriteria'  , @(t,sysList)t>70,...
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1, ... 
    'ExtraLogs'         , {ILog('e',@(t,agent,varargin)e(t,agent.x))},...
    'StepPlotFunction'  , @ex02StepPlotFunction ...  %% <<< difference from ex01 ( plot estimate )
    );

log = va.run();