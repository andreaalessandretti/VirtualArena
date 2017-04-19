clc; close all; clear all;

dt = 0.2; 

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
   

%% <<< BEGIN difference from ex02  

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

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
    );

%% <<< END difference from ex02 

va = VirtualArena(dtRealSystem,... % <<< Discrete time simulation
    'StoppingCriteria'  , @(t,sysList)t>70/dt,...
    'PlottingStep'      , 1/dt, ... 
    'ExtraLogs'         , {ILog('e',@(t,agent,varargin)e(t*dt,agent.x))},...  % <<< Log Lyapunov function value
    'StepPlotFunction'  , @ex02StepPlotFunction ... 
    );

log = va.run();

figure; grid on;xlabel('time [s]');ylabel('Estimated V(t)'); hold on
for ii = 1:length(log)
    plot(log{ii}{1}.time*dt,0.5*sum(log{ii}{1}.e.*log{ii}{1}.e)); 
end
a = axis; a(2)=70; axis(a);