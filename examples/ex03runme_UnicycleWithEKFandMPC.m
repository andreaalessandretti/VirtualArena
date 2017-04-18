%% Example MPC controller

clc;close all;clear all;

dt = 0.1;

%% Unicycle Model
sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)],...
<<<<<<< HEAD
<<<<<<< HEAD
    'OutputEquation', @(t,x) x(1:2), 'ny', 2,... %% <<< difference from ex01 ( e.g., GPS )
=======
    'OutputEquation', @(t,x,u) x(1:2),'ny',2, ... % GPS
>>>>>>> InlineClasses
=======
    'OutputEquation', @(t,x,u) x(1:2),'ny',2, ... % GPS
>>>>>>> InlineClasses
    'nx',3,'nu',2 ...
);

desiredPosition      = [0;0];


realSystem = CtSystem(...
    'StateEquation', @(t,x,u) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)] + [0.1*randn(2,1);randn(1,1)*pi/8],...
    'OutputEquation', @(t,x) x(1:2), 'ny', 2,...
    'nx',3,'nu',2 ...
);

realSystem.stateObserver = EkfFilter(DtSystem(sys,dt),...
                 'StateNoiseMatrix'  , diag(([0.1,0.1,pi/8])/3)^2,...
                 'OutputNoiseMatrix' , diag(([0.1,0.1])/3)^2,...
                 'InitialCondition'  , repmat({[[1;-1;0];                  %xHat(0)
                                        10*reshape(eye(3),9,1)]},1,10*4));  %P(0)

realSystem.initialCondition = repmat({[1;1;pi/2],-[1;1;-pi/2],[1;-1;-pi/2],[-1;1;-pi/2]},1,10);
                                 

%% <<< BEGIN difference from ex02   

mpcOp = ICtMpcOp( ...
    'System'               , sys,...
    'HorizonLength'        , 0.5,...
    'StageConstraints'     , BoxSet( -[1;pi/4],4:5,[1;pi/4],4:5,5),... % on the variable z=[x;u];
    'StageCost'            , @(t,x,u,varargin) (x(1:2)-desiredPosition)'* (x(1:2)-desiredPosition),...
    'TerminalCost'         , @(t,x,varargin) (x(1:2)-desiredPosition)'* (x(1:2)-desiredPosition)...
    );

dtMpcOp = DiscretizedMpcOp(mpcOp,dt);

<<<<<<< HEAD
<<<<<<< HEAD
realSystem.controller = MpcController(...
=======
=======
>>>>>>> InlineClasses
dtSys   = DiscretizedSystem(sys,dt);

dtSys.controller = MpcController(...
>>>>>>> InlineClasses
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
    );

%% <<< END difference from ex02 

<<<<<<< HEAD

va = VirtualArena(realSystem,...
    'StoppingCriteria'  , @(t,sysList)t>6,...
=======
dtSys.stateObserver = EkfFilter(dtSys,...
                 'StateNoiseMatrix'  , diag(([0.1,0.1,pi/4])/3)^2,...
                 'OutputNoiseMatrix' , diag(([0.1,0.1])/3)^2,...
                 'InitialCondition'  , [[1;-1;0];                  %xHat(0)
                                        10*reshape(eye(3),9,1)]);  %P(0)

dtSys.initialCondition = [1;1;0];

va = VirtualArena(dtSys,...
    'StoppingCriteria'  , @(t,sysList)norm(sysList{1}.x(1:2)-desiredPosition)<0.1,...
>>>>>>> InlineClasses
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1, ... 
    'StepPlotFunction'  , @ex02StepPlotFunction ...  %% <<< difference from ex01 ( plot estimate )
    );

log = va.run();

figure(2)
for kk = 1:length(log)
subplot(2,1,1)
plot(log{kk}{1}.time,log{kk}{1}.inputTrajectory(1,:));hold on 
subplot(2,1,2)
plot(log{kk}{1}.time,log{kk}{1}.inputTrajectory(2,:));hold on 
end

subplot(2,1,1)
axis([0,6,-1.1,1.1]);
ylabel('v_f');
subplot(2,1,2)
axis([0,6,-1,1]);
xlabel('Time (s)');
ylabel('\omega');
