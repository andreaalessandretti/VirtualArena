
<div align="center"><img src=".\logo.jpg" width="200 align="middle"> </div>

<<<<<<< HEAD
VirtualArena is Object-Oriented Matlab toolkit for Control Design and System Simulation.
=======
VirtualArena is Object-Oriented Matlab Toolkit for Control Design and System Simulation.
>>>>>>> InlineClasses

## Why VirtualArena?

#### Example: Control of a Double Integrator
#####Common Coding Approach

```matlab
Tend   = 10;
dt     = 0.1;     % Discretization step
n      = floor(Tend/dt);
x      = zeros(2,n);
u      = zeros(1,n-1);
kSpan  = 1:n-1;

x(:,1) = [1;1];   % Initial condition 
 
for k = kSpan
    u(:,k)    = -[1,1.7321]*x(:,k);            % Control law
    f         = [0,1;0,0]*x(:,k)+[0;1]*u(:,k); % Continuous time dynamical system
    x(:,k+1)  = x(:,k) + dt*f;                 % Euler forward method
end


figure(2)
subplot(3,1,1)
plot(kSpan*dt,x(1,1:end-1));grid on;xlabel('t');ylabel ('x_1')
subplot(3,1,2)
plot(kSpan*dt,x(2,1:end-1));grid on;xlabel('t');ylabel ('x_2')
subplot(3,1,3)
plot(kSpan*dt,u);grid on;xlabel('t');ylabel ('u')
```
#####VirtualArena

```matlab
sys = CtSystem(...
    'StateEquation', @(t,x,u) [0,1;0,0]*x+[0;1]*u,...
    'nx',2,'nu',1 ...
);

sys.initialCondition = [1;1];

sys.controller = InlineController(@(t,x)-[1,1.7321]*x);

va = VirtualArena(sys,...
    'StoppingCriteria'  ,@(t,as)t>10,...
    'DiscretizationStep',dt...
    );

va.run();
```
### Key Benefits of VirtualArena
- **Modularity & reusability & maintenance.** VirtualArena promote the development of a modular structure, where each object (e.g., a controller, a dynamical model...) is self-contained, and interchangeable. Once an object is developed, the knowledge of its implementation is not necessary for its use. This abstraction allows to reuse/share components and easily maintain the architecture by simply replacing selected blocks.

- **Collaborative design & test.** When multiple developers are involved in the design of a control architecture, it is required a formalization of common interfaces to connect the different components of the architecture. VirtualArena defines such interfaces, facilitating the assignment of the design of modules to different developers.

- **Dissemination & extensibility.** The effort and the technological and theoretical background required for the implementation of advanced control strategies are the main obstacles slowing down the dissemination of new research results. VirtualArena, allowing the development of modular controllers, promotes the sharing of new algorithms among the users and provides functions to install external modules from third-party githubs.

- **Many control-oriented common functions implemented.** There are many common components/procedures required in the simulation/design of control systems (e.g., discretization procedures, linearization procedures, EKF design, data logging architecture, ...). VirtualaArena comes with an increasing number of predefined functionalities devoted to make the life of the developer easier.

## Getting Started
Open Matlab on the folder where you want to create the VirtualArena folder and run the following code

```matlab
urlwrite('https://github.com/andreaalessandretti/VirtualArena/archive/master.zip','master.zip');
unzip('master.zip');
movefile('VirtualArena-master','VirtualArena');
cd VirtualArena/;
addPathsOfVirtualarena;
```

## Features
In this section, we list some of the features of VirtualArena. We refer to the Matlab documentation for the use of specific functions.

###Simulation
- Implementation of different time discretization methods (Euler forward, Runge-Kutta, ...).
- Easy-to-use logging management system.
- Easy-to-use system to run multiple simulations with different initial conditions and initial settings (e.g., different initial conditions, different controller parameters, ...). 
- Simulation of a communication/sensor network of multiple vehicles.
- Simultaneous simulation of a network of multiple vehicles.

###System Manipulation
- Discretization of continuous-time dynamical models.
- Linearization of continuous-time and discrete-time dynamical models.

###State Estimation
- Symbolic Hessian and Jacobian computation or estimation via sampling.
- Automatic generation of Extended Kalman Filter for discrete-time dynamical systems.
- Automatic generation of Extended Kalman-Bucy Filter for continuous-time dynamical systems.

###Model Predictive Control
- Interface to solve discrete-time MPC problems using Fmincon.
- Implementation of different warm-start strategies for MPC controllers.

###Motion Control of Underactuated Vehicle
- Different representations of attitude using quaternions and rotation matrices.
- Generic dynamical model representing a variety of vehicles.

## Examples
In this section, we illustrate the use of `VirtualArena` using some examples, which can be found in the folder `\VirtualArena\examples`.

### Ex 01: Control of a Unicycle Model
This example addresses the control of unicycle system. We will see how to :
- Define controller in a separate file
- Use custom StagePlotFunction
- Use custom StoppingCriteria 

Main file ```ex01runme_Unicyle.m```:

```matlab
clc; close all; clear all;

dt = 0.1;

%% Unicycle Model
sys = CtSystem(...
    'StateEquation', @(t,x,u) [
    u(1)*cos(x(3));
    u(1)*sin(x(3));
    u(2)],...
    'nx',3,'nu',2 ...
);

desiredPosition      = [0;0];

sys.controller       = UniGoToPoint(desiredPosition);

sys.initialCondition = [1;1;0];

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,sysList)norm(sysList{1}.x(1:2)-desiredPosition)<0.1,...
    'DiscretizationStep', dt,...
    'PlottingStep'      , 1, ...
    'StepPlotFunction'  , @ex01StepPlotFunction ...
    );

log = va.run();
```

The controller is defined in ```UniGoToPoint.m```:

```matlab
classdef UniGoToPoint < Controller
    %UniGoToPoint state-feedback go-to-position Controller for Unicycle model
    %
    % controller = UniGoToPoint (point)
    %
    % point - desired 2d position
    %
    % See also Controller
    
    properties
        point;  
    end
    
    methods
        
        function obj = UniGoToPoint(point)
            
            obj.point = point;
            
        end
        
        function u = computeInput(obj,t,x)
            
            R = [cos(x(3)),-sin(x(3));
                 sin(x(3)), cos(x(3))];      % Body to inertia rotation matrix
            
            err = R'*(obj.point - x(1:2));   % Position error in body frame
            
            thetaErr = atan2(err(2),err(1)); % Angle error
            
            u = [norm(obj.point-x(1:2));
                 thetaErr];
            
        end
        
    end
end
```
The `StepPlotFunction` is defined in ```ex01StepPlotFunction.m```:

```matlab
function h = ex01StepPlotFunction(sysList,log,plot_handles,k)

logX = log{1}.stateTrajectory(:,1:k); hold on;
h    = plot(logX(1,:),logX(2,:));
grid on

end

```
### Ex 02: Output Feedback and EKF
Here, the previous example is extended by making available only the position of the unicycle and designing an Extended Kalman Filter to estimate its sate.

In order to define an output of the system, we can use the parameter `OutputEquation` in the definition of the system.

```matlab
***
sys = CtSystem(...
    ***
    'OutputEquation', @(t,x) x(1:2), 'ny', 2,... %% e.g., GPS 
    ***
);
***
```

At this point, running `VirtualArena` would generate and error, since the controller requires the 3-dimentional state when it is receiving the 2-dimentional output vector. In order to overcome this problem, we add an Extended Kalman Filter in the system.

```matlab
dtSys   = DtSystem(sys,dt);

sys.stateObserver = EkfFilter(dtSys,...
                 'StateNoiseMatrix'  , diag(([0.1,0.1,pi/4])/3)^2,...
                 'OutputNoiseMatrix' , diag(([0.1,0.1])/3)^2,...
                 'InitialCondition'  , [[1;-1;0];                  %xHat(0)
                                        10*reshape(eye(3),9,1)]);  %P(0)
```
Since now we also have an estimate of the state, we update the `StepPlotFunction` as follows:

```matlab
function h = ex02StepPlotFunction(sysList,log,plot_handles,k)

logX    = log{1}.stateTrajectory(:,1:k); hold on;
logHatX = log{1}.observerStateTrajectory(:,1:k); hold on;
h(1)    = plot(logX(1,:)   ,logX(2,:));
h(2)    = plot(logHatX(1,:),logHatX(2,:),'--');
grid on

end
```
**Note that:**

- `DtSystem(sys,dt)` discretized the continuous time model
- The `EkfFilter` automatically computed the linearized system and used it to design the Extended Kalman Filter.

### Ex 03: A simple MPC controller
Building on the previous examples, we replace the controller `UniGoToPoint` with a simple MPC controller.

```matlab
mpcOp = CtMpcOp( ...
    'System'               , sys,...
    'HorizonLength'        , 0.5,...
    'StageConstraints'     , BoxSet( -[1;pi/4],4:5,[1;pi/4],4:5,5),... % on the variable z=[x;u];
    'StageCost'            , @(t,x,u,varargin) (x(1:2)-desiredPosition)'* (x(1:2)-desiredPosition),...
    'TerminalCost'         , @(t,x,varargin) (x(1:2)-desiredPosition)'* (x(1:2)-desiredPosition)...
    );

dtMpcOp = DtMpcOp(mpcOp,dt);

dtSys   = DtSystem(sys,dt);

dtSys.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
    );

sys = dtSys;
```
### Ex 04: Controlling a real system
At this point, we implemented an MPC controller that uses an estimate from an Extended Kalman Filter to compute the optimal input. In this example, we show an simple approach to test this output-feedback control strategy on a real system. A remote python script emulates the real system.

Toward this goal, it is enough to create the system in `ex04RemoteUnicycle.m` and attach the previusly desinged controllers and observer to it.

```matlab
realSystem = ex04RemoteUnicycle(...
            'RemoteIp','127.0.0.1','RemotePort',20001,...
            'LocalIp' ,'127.0.0.1','LocalPort',20002);

realSystem.stateObserver =  EkfFilter(DtSystem(sys,dt),...
	***

realSystem.controller = MpcController(...
	***

va = VirtualArena(realSystem,...
	***
	            
```
Note that, as expected, the observer is designed on the model `sys` and then attached on the real system `realSystem`.
At this point, before running the file `ex04runme_UnicycleWithEKFandMPCRealSystem.m` run on the terminal the python script `ex04RealSystem.py` that emulate the real system `python ex04RealSystem.py `.

**Remark:** Note that this is just an example to show the structure of VirtualArena. Indeed, in most cases this is not the best way of using MPC controllers on real systems.

### Ex 05: Distributed control - basic consensus algorithm

VirtualArena supports the simulation of a network of systems. In this example, we see how to set up and run a consensus control scheme of a set of single integrators.

```matlab
N = 5;

for i = 1:N
    
    v{i}                  = CtSystem('StateEquation',@(t,x,u)u,'nx',1,'nu',1);    
    v{i}.controller       = ex5BasicConsensusController();
    v{i}.initialCondition = i;
    
end

%% Network

% Adjacency matrix of a loop
% Aij = 1 if agent i measures agent j
A            = zeros(N);
A(1,4)       = 1;
A(2:N,1:N-1) = eye(N-1);

%A Sensor defines what to measure, in this case the state
s1 = ex5StateSensor();

a = VirtualArena(v,...
    'StoppingCriteria'  , @(t,as)t>10,...
    'SensorsNetwork'    , {s1,A},...
    'DiscretizationStep', 0.1,...
    'PlottingStep'      , 1);

ret = a.run();

%Show the final state to see that the system reached consensus
for i=1:length(ret)
    finalState = ret{i}.stateTrajectory(1,end)'
end
```
A sensor object, subcalss of `Sensor` defines what kind of measurement we are taking. Since we want to measure the state of our neighborhoods, the implementation will be as follows:

```matlab
classdef ex5StateSensor < Sensor
    methods (Static)
        
        function  measurement = sense(agentId,agent,detectableAgentsList,detectableAgentsIds)
            measurement = {};
            nDetectables = length(detectableAgentsIds);
            for i = 1 : nDetectables
                measurement{i} = detectableAgentsList{i}.x;
            end 
        end
        
    end
end
```

The `Sensor` mounted on `agent`, with associated `agentId`, 
 `sense`s the `detectableAgentsList` with associated `detectableAgentsIds` and provides a `measurement`. In this case, the `measurement` is a call containing all the states of the detectable agents. The adjacency matrix defines the detected agents

A network based controller, other than time and state, has as input also the readings from the network, which in this case are the state of the neighborhooding systems.

```matlab
classdef ex5BasicConsensusController < Controller
    methods
        
        function u = computeInput(obj,t,x,readings)
            nNeigh = length(readings{1});
            u = 0;
            for i =1:nNeigh
                u = u+(readings{1}{i} -x)/nNeigh;
            end
        end
    end
    
end
```

## Reference

If you like VirtuaArena, you can reference the associated Ph.D. thesis with BibTeX entry:

```
@phdthesis{Alessandretti2016,
author = {Alessandretti, A.},
school = {Instituto Superior T{\'{e}}cnico - Ecole Polytechnique F{\'{e}}d{\'{e}}rale de Lausanne},
title = {{Continuous-time Model Predictive Control for Economic Optimization: Theory, Design, and Applications to Motion Control of Underactuated Vehicles}},
year = {2016}
}
```
