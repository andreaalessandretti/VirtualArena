%% Using a Remote System
%Before running this file run in the terminal the python system:
%python ex04RealSystem.py 

clc;close all;clear all;

dt = 0.1;

%% Unicycle Model
sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)],...
<<<<<<< HEAD
    'OutputEquation', @(t,x) x(1:2), 'ny', 2,... %% <<< difference from ex01 ( e.g., GPS )
=======
    'OutputEquation', @(t,x,u) x(1:2),'ny',2, ... % GPS
>>>>>>> InlineClasses
    'nx',3,'nu',2 ...
);

desiredPosition      = [0;0];

%% <<< BEGIN difference from ex03                                  
realSystem = ex04RemoteUnicycle(...
            'RemoteIp','127.0.0.1','RemotePort',20001,...
            'LocalIp' ,'127.0.0.1','LocalPort',20002);
%% <<< END difference from ex03 


<<<<<<< HEAD
realSystem.stateObserver = EkfFilter(DtSystem(sys,dt),... %% <<< the filter is applied to the real system  
                 'StateNoiseMatrix'  , diag(([0.1,0.1,pi/4])/3)^2,...
                 'OutputNoiseMatrix' , diag(([0.1,0.1])/3)^2,...
                 'InitialCondition'  , [[1;-1;0];                  %xHat(0)
                                        10*reshape(eye(3),9,1)]);  %P(0)
  
mpcOp = CtMpcOp( ...
=======
mpcOp = ICtMpcOp( ...
>>>>>>> InlineClasses
    'System'               , sys,...
    'HorizonLength'        , 0.5,...
    'StageConstraints'     , BoxSet( -[1;pi/4],4:5,[1;pi/4],4:5,5),... % on the variable z=[x;u];
    'StageCost'            , @(t,x,u,varargin) (x(1:2)-desiredPosition)'* (x(1:2)-desiredPosition),...
    'TerminalCost'         , @(t,x,varargin)   (x(1:2)-desiredPosition)'* (x(1:2)-desiredPosition)...
    );

dtMpcOp = DiscretizedMpcOp(mpcOp,dt);

<<<<<<< HEAD
=======
dtSys   = DiscretizedSystem(sys,dt);

>>>>>>> InlineClasses
realSystem.controller = MpcController(...  %% <<< the controller is applied to the real system 
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp) ...
    );
realSystem.controller.mpcOpSolver.symbolizeProblem(dtMpcOp);

va = VirtualArena(realSystem,...%% <<< difference from ex03
    'StoppingCriteria'  , @(t,sysList)norm(sysList{1}.stateObserver.x(1:2)-desiredPosition)<0.01,...
    'DiscretizationStep', dt,...
    'ExtraLogs'         , {MeasurementsLog(sys.ny)},... %% <<< difference from ex03
    'StepPlotFunction'  , @ex04StepPlotFunction ...
    );

log = va.run();


plot(log{1}.measurements(1,:),log{1}.measurements(2,:),'o')