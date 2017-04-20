

This section introduces VirtualArena toolkit by illustrating all the steps for the implementation a common control architecture.

## Model and state feedback controller definition

Let <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode" align=middle width=8.4843pt height=22.38192pt/> be an inertial coordinate frame and <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode" align=middle width=13.243725pt height=22.38192pt/> be a body coordinate frame attached to an UAV. The pair <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/db13ad244a5538f130b5418ab8fa6f0a.svg?invert_in_darkmode" align=middle width=143.1342pt height=24.56553pt/> denote the configuration of the vehicle, position and orientation, where  <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/9f17a8869fae32435d1b360980e0d59b.svg?invert_in_darkmode" align=middle width=31.213545pt height=24.56553pt/> is the rotation matrix from  body to inertial coordinates. Now, let <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/14a12088ca08e01cd62e5a13a16933b4.svg?invert_in_darkmode" align=middle width=177.169245pt height=24.56553pt/> be the twist that defines the velocity of the vehicle, linear and angular, where the matrix <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/181743204147e8724f68589b3beca646.svg?invert_in_darkmode" align=middle width=54.00087pt height=24.56553pt/> is the skew-symmetric matrix associated to the angular velocity <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/3c745e329d420cb1125671f500fe1c0c.svg?invert_in_darkmode" align=middle width=61.31334pt height=24.56553pt/>, defined as
<img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/427815ec77e3d63d5f2d1222ee350dec.svg?invert_in_darkmode" align=middle width=144.125685pt height=47.66652pt/>
The kinematic model of the body frame satisfies
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/2b3f0ac154393d2ead5a53cacac3fe2b.svg?invert_in_darkmode" align=middle width=700.1643pt height=93.103725pt/></p>
with <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/738d8637c8da09da7bda826cf834687d.svg?invert_in_darkmode" align=middle width=204.130245pt height=32.55549pt/>
that only consists of forward and angular velocity. The planar model \eqref{eq:constrained_model} captures the under-actuated nature of a wide range of vehicles and is often used for the control of a set of UAV moving in at 2-D plane, i.e., relying on an on-board inner controller for altitude stabilization. 


Consider the \emph{control point} that we wish to control
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/ffecc0b18ccdf0bf36b10703c3a2e02f.svg?invert_in_darkmode" align=middle width=418.7601pt height=16.376943pt/></p> that is a constant point in the body frame placed at a constant distance <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/9a5e88b330a69bfc993b1fb46abe550e.svg?invert_in_darkmode" align=middle width=119.605695pt height=26.70657pt/> from the center of rotation of the body frame <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/2a540f4203b423259f53da4c007171ce.svg?invert_in_darkmode" align=middle width=13.243725pt height=22.38192pt/>. For a desired trajectory <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/11db30fced2bb8ca4d18616e629c2851.svg?invert_in_darkmode" align=middle width=84.12657pt height=26.70657pt/> for the control point \eqref{eq:cc}, it is possible to show (see \cite{Alessandretti2013,Alessandretti2017}) that the control law
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/a58654212f099659514aa656fe63baab.svg?invert_in_darkmode" align=middle width=434.5902pt height=42.926895pt/></p> with <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/0134d782b8beb449e877cf9d7aff5c38.svg?invert_in_darkmode" align=middle width=115.72935pt height=47.66652pt/> for any <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/3107aa0f49a02cf789600b9ecaca9504.svg?invert_in_darkmode" align=middle width=44.08041pt height=22.74591pt/> stabilizes the origin of the tracking error space <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/a959dddbf8d19a8a7e96404006dc8941.svg?invert_in_darkmode" align=middle width=37.68402pt height=21.10812pt/>. The following code is used to simulated the closed-loop \eqref{eq:constrained_model} with \eqref{eq:uinp} with associated plots displayed in Fig. \ref{fig:ex1}.

```matlab
dt = 0.05;

%% Unicycle Model
sys = ICtSystem(... |\label{line:isys}|
'StateEquation', @(t,x,u,varargin) [
u(1)*cos(x(3));
u(1)*sin(x(3));
u(2)],...
'nx',3,'nu',2 ...
);

sys.initialCondition = {[15;15;-pi/2],-[15;15;-pi/2],[15;-15;pi],[-15;15;-pi/2]};|\label{line:ini}|

sys.controller = TrackingController_ECC13(...
@(t) 10*[sin(0.1*t); cos(0.1*t)],...% c
@(t)    [cos(0.1*t);-sin(0.1*t)],...% cDot
eye(2)                          ,...% K
[1;0] ); % epsilon

va = VirtualArena(sys,...
'StoppingCriteria'  , @(t,sysList)t>70,...
'DiscretizationStep', dt,...
'PlottingStep'      , 1, ...
'StepPlotFunction'  , @ex01StepPlotFunction);

log = va.run();
```

It is worth noticing that the simulation is automatically executed multiple times, one for each initial condition. This is generally a desirable feature to test via simulation the behavior of the controller in different scenarios.  The letter `I` in the name `ICtSystem`, and in many other default classes in VirtualArena, stands for ``inline'', i.e., that can be defined inline without the need of creating a separate file. This is in contrast, for instance, to the controller `TrackingController_ECC13`. In fact, due to its complexity, the latter is not declared using the class `IController`, available in VirtualArena, but it is defined in the separate file `TrackingController_ECC13.m` reported next.
```matlab
classdef TrackingController_ECC13 < Controller

properties
epsilon,cdes,cdesDot,K,PinvE
end

methods
function obj = TrackingController_ECC13(cdes,cdesDot,K,epsilon)

obj = obj@Controller();

obj.cdes      = cdes;
obj.cdesDot   = cdesDot;
obj.K       = K;
obj.epsilon = epsilon;

E   = [[1;0],[-epsilon(2);epsilon(1)]];

if not(rank(E)==size(E,1))
error('System not controllable, try a different epsilon');
end

obj.PinvE = inv(E);
end

function u = computeInput(obj,t,x)          
R = [cos(x(3)),-sin(x(3));
sin(x(3)),cos(x(3))];
e = obj.computeError(t,x);
u = -obj.PinvE*(obj.K*e-R'*obj.cdesDot(t));          
end

function e = computeError(obj,t,x)
p = x(1:2);
R = [cos(x(3)),-sin(x(3));
sin(x(3)),cos(x(3))];
e = R'*(p-obj.cdes(t))+obj.epsilon;            
end end end
```

The `StepPlotFunction` is defined in the separate file `ex01StepPlotFunction.m`.
```matlab
function h = ex01StepPlotFunction(sysList,log,plot_handles,k)
logX = log{1}.stateTrajectory(:,1:k);hold on;
h    = plot(logX(1,:),logX(2,:));grid on; 
end
```

By  default, VirtualArena logs time, state trajectories, input trajectories, and whenever applicable, the trajectories associated with the internal state of the controller and state observer. Each log is associated with an object subclass of `Log`, e.g., by default VirtualArena loads the loggers, `TimeLog()`, `StateLog()`, `InputLog()`,`ControllerStateLog()`, and `ObserverStateLog()`, all subclass of `Log`. Custom logs can be defined for specific applications, e.g., to log tracking error, Lyapunov function value, etc.

<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/208858fe8cbd1a5b26a1a0205c53c72b.svg?invert_in_darkmode" align=middle width=698.0061pt height=68.17305pt/></p>

\subsection{ Output feedback and EKF design} \label{sec:ex2}

The previous example considers the state feedback case. This section assumes that the state of the vehicle is observed with the observation model <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/6e02a662632649a83aa133134b251423.svg?invert_in_darkmode" align=middle width=76.031835pt height=24.56553pt/>, i.e., that captures, for instance,  the case where only the position, and not the heading, is available for control design, e.g., a GPS measurement. An Extended Kalman Filter (EKF, see, e.g., \cite{Anderson2005}) is designed on the vehicle model to estimate the state of the system, and then the same state feedback controller of the previous section is adopted for the motion control.

One way to define an output of the system in VirtualArena is to add the `OutputEquation` in the inline definition of the system as follows.
```matlab
...
sys = ICtSystem(...
...
'OutputEquation',@(t,x,varargin)x(1:2),...
'ny',2,...
...
);
...
```
The automatic generation of the EKF with initial covariance matrix <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/2438401579804bb43c568b5abb27cd4a.svg?invert_in_darkmode" align=middle width=86.626485pt height=22.38192pt/>, state noise covariance matrix <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/ab700b9d0884184c8eef44ac327e4cd1.svg?invert_in_darkmode" align=middle width=175.027545pt height=26.70657pt/>, and output noise covariance matrix <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/495ff644277a1194d51d245811d32fb9.svg?invert_in_darkmode" align=middle width=141.05157pt height=26.70657pt/>, where <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/d2b2bb8446799af30401fb363399038e.svg?invert_in_darkmode" align=middle width=57.15105pt height=24.56553pt/> denotes a diagonal matrix <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/78ec2b7008296ce0561cf83393cb746d.svg?invert_in_darkmode" align=middle width=14.01378pt height=22.38192pt/> with <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/5b778222f1512b9b2be0c2811150be20.svg?invert_in_darkmode" align=middle width=59.577705pt height=22.38192pt/> and <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/0136874899d32da31577e61071524619.svg?invert_in_darkmode" align=middle width=33.62601pt height=22.38192pt/> represents the identity matrix of size <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/55a049b8f161ae7cfeb0197d75aff967.svg?invert_in_darkmode" align=middle width=9.83004pt height=14.10255pt/>, is obtained as follows.
```matlab
...
% System with state and input noise covariance matrices
Q = diag(([0.1,0.1,pi/4])/3)^2;
R = diag(([0.1,0.1])/3)^2;

% Model with additive noise 
realSystem = ICtSystem(... |\label{line:realSystem}|
'StateEquation',  ...
@(t,x,u,varargin) ...
sys.f(t,x,u,varargin{:})+chol(Q)*randn(3,1),...
'OutputEquation', ...
@(t,x,varargin)...
sys.h(t,x,varargin{:})+chol(R)*randn(2,1),...
'ny', 2,'nx',3,'nu',2);

% Initial conditions for the system ...
realSystem.initialCondition = ...
repmat({[15;15;-pi/2],-[15;15;-pi/2],...
[15;-15;-pi],[-15;15;-pi/2]},1,10); 

% ... and associated initial conditions for the observer
for ii = 1:length(realSystem.initialCondition)
x0Filter{ii} = [
realSystem.initialCondition{ii} ...
+5*randn(3,1);        %xHat(0)
10*reshape(eye(3),9,1) ]; %P(0)
end

realSystem.stateObserver = EkfFilter(...
DiscretizedSystem(sys,dt),... |\label{line:dt}|
'StateNoiseMatrix' , dt*Q,...
'OutputNoiseMatrix', R,...
'InitialCondition' , x0Filter);

...

realSystem.controller = TrackingController_ECC13(... % <<< attached to the realSystem
...
va = VirtualArena(realSystem,... % <<< simulate realSystem
```
Since the Extended Kalman Filter is defined for discrete-time system, in Line \ref{line:dt}  `DiscretizedSystem(sys,dt)` is used to discretized the system using Runge-Kutta 4, which is the default discretization method. Different, and possibly custom, discretization methods can be specified, e.g., `DiscretizedSystem(sys,dt,EulerForward())`. The object `EkfFilter` automatically computes the linearization of the system that will be evaluated by VirtuaArena around the closed-loop state and input trajectory. The processes of system discretization, linearization, and observer design for nonlinear systems are often involving and time-consuming steps toward the validation of control law. In this example, VirtualArena aims to minimize the associated effort to directly target the analysis of the control law.

In order to observe the effect of the noise the system is initialized utilizing the same set of initial conditions from the previous example repeated ten times with added noise, leading to a total of forty simulations. 
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/bf641eb020009a5d3561ed10eba985a4.svg?invert_in_darkmode" align=middle width=698.05065pt height=107.62521pt/></p>

\subsection{Model Predictive Control implementation}\label{sec:ex3}

Building on the previous examples, the controller `TrackingController_ECC13` is replaced with an Model Predictive Control (MPC) control law designed to drive the error vector \eqref{eq:defe} to the origin. Specifically, we consider the MPC tracking controller from \cite{Alessandretti2013} where the terminal cost and terminal set is re-designed as in \cite{Alessandretti2017} to obtain global convergence. The following definition of open-loop MPC problem is considered in the definition of the MPC controller.
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/84a758f12659bd97e82b924bb5a08772.svg?invert_in_darkmode" align=middle width=700.1643pt height=419.4432pt/></p>

In order to make explicit the dependence of the optimal solution to the parameters of the MPC open-loop optimization problem, the term <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/cb38e6a81d80f9edf048d3a6c84f278b.svg?invert_in_darkmode" align=middle width=68.39514pt height=31.42161pt/> denotes the <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/77a3b857d53fb44e33b53e4c8b68351a.svg?invert_in_darkmode" align=middle width=5.642109pt height=21.60213pt/>-th input of the optimal input sequence <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/eeab960d17bcfbfb4863c3ff936ec204.svg?invert_in_darkmode" align=middle width=6.7100715pt height=22.59873pt/> computed by solving <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/e4ec7df89a71c711465119ac2015a1cf.svg?invert_in_darkmode" align=middle width=51.239595pt height=31.42161pt/>. 

A standard MPC controller is obtained by solving at every time step <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/f815f0ac844d327ebb4a77c246c121d2.svg?invert_in_darkmode" align=middle width=45.96735pt height=22.74591pt/> the open-loop MPC problem and applying the first input of the optimal input trajectory to the system as follows
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/5201e628eaaae62473e381cb432f1b67.svg?invert_in_darkmode" align=middle width=491.03505pt height=16.376943pt/></p>

Following \cite{Alessandretti2017}, choosing the performance index with
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/788b12c67ba774665e0837e9523e373e.svg?invert_in_darkmode" align=middle width=576.51165pt height=18.269295pt/></p>
the input constraint set 
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/5a07bba0b8162ac813bad8c831ea3a2c.svg?invert_in_darkmode" align=middle width=214.04295pt height=39.30498pt/></p> and the state constraint sets <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/2496b7bd09c68227844d2d2dd7b152b6.svg?invert_in_darkmode" align=middle width=148.34787pt height=26.70657pt/> satisfies the sufficient conditions for global convergence of the error vector to the origin. In VirtualArena, this MPC controller can be defined as follows:
```matlab
...
auxiliaryControlLaw=TrackingController_ECC13( ... )

e=@(t,x)auxiliaryControlLaw.computeError(t,x);

mpcOp = ICtMpcOp( ...
'System'           , sys,...
'HorizonLength'    , 2*dt,...
'StageConstraints' , ...
BoxSet( -[1.1;1.1],4:5,[1.1;1.1],4:5,5),... % on the variable z=[x;u];
'StageCost'        , ...
@(t,x,u,varargin) e(t,x)'* e(t,x),...
'TerminalCost'     , ...
@(t,x,varargin) 0.3333*(e(t,x)'* e(t,x))^(3/2)...
);

dtMpcOp = DiscretizedMpcOp(mpcOp,dt);|\label{line:dtMpcOp}|

dtRealSystem=DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
'MpcOp'       , dtMpcOp ,...
'MpcOpSolver' , FminconMpcOpSolver(...
'MpcOp', dtMpcOp,...
'UseSymbolicEvaluation',1...
) ...
);
...

va = VirtualArena(dtRealSystem,... % <<< Discrete time simulation
'StoppingCriteria'  , @(t,sysList)t>70/dt,...
'PlottingStep'      , 1/dt, ... 
'StepPlotFunction'  , @ex02StepPlotFunction ... 
);
```

The `MpcOpSolver` utilizes in this simulation is `FminconMpcOpSolver` that is designed, building on the `fmincon` function of MATLAB, to solve discrete time MPC optimization problems. Because of this, in Line \ref{line:dtMpcOp} the MPC optimization problem is discretized before being passes as parameter to the `MpcController` in charge of implementing the receding horizon strategy \eqref{eq:kmpc}.

It is worth noticing that the implementation of MPC scheme is generally a complex task that motivated a wide set of tools from the technical literature. In this example, VirtualArena is used to design in few lines of code an MPC controller for a constrained nonlinear system. Whereas in this implementation considers the `FminconMpcOpSolver` object to solve the MPC problem, VirtualArena defines the general interface `MpcOpSolver` that can be used to interface existing MPC solver from the literature, such as ACADO \cite{OCA:OCA939} and FORCES \cite{FORCESPro}, or to custom ones.

<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/93534aedc7121c01da57f3bc1f26149f.svg?invert_in_darkmode" align=middle width=699.2271pt height=110.809545pt/></p>

\subsection{Controlling remote system via UDP over LAN network}\label{sec:ex4}
In the previous sections, we were able with a small effort to start from a model of a UAV, generate an Extended Kalman Filter able to estimate the full state of the vehicle only using position measurements, and design and simulate an MPC controller to steer the position of the control point to a desired trajectory. In this section, we use the same code developed in the previous section to control a remote system over an Internet connection.

A basic network control strategy (NCS) consists of reading the measurements received from a remote sensor on board of the vehicle, computing the control input, sending the control input to a remote actuator, and iterating the process. Since VirtualArena captures the dynamics of the vehicle in a dedicated object, in order to implement a MATLAB-in-the-loop NCS strategy it is enough to replace the `realSystem` in the previous example with an object that, instead of simulating the dynamical model of the system, communicate with the real one.

This is done in the system `ex04RemoteUnicycle.m` available at the GitHub page of the toolbox. Thus, it is enough to attach the previously designed controllers and observer to the `ex04RemoteUnicycle` as follows.
```matlab
realSystem = ex04RemoteUnicycle(...
'RemoteIp','127.0.0.1','RemotePort',20001,...
'LocalIp' ,'127.0.0.1','LocalPort',20002);

realSystem.stateObserver =  EkfFilter(DtSystem(sys,dt),...
...
realSystem.controller = MpcController(...
```
At this point, before running the file `ex04runme_UnicycleWithEKFandMPCRealSystem.m` we can run the python script `ex04RealSystem.py` that emulates the real system.

<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/5f0d09477f0149738aab9b37b6b21564.svg?invert_in_darkmode" align=middle width=698.08035pt height=71.35755pt/></p>

Fig. \ref{fig:plotpy} shows the associated closed-loop trajectory. Notice that, in contrast to the previous examples, the real state of the vehicle is not displayed, since not available, and only the measurements the state estimate is shown.

This section only presents an illustrative example of a basic NCS. Although, for the control of a real system with fast dynamics, delays, and dropouts in the communication link, it is opportune to design a robust NCS schemes. 

Model Predictive Control, with its ability to generate future state and input prediction, is particularly suitable for this kind of applications. In fact, instead of sending only the input associated with the current time, it is possible to send the whole state and input predicted trajectory to a ``smart'' actuator that will apply open-loop the input trajectory to the system (or track the associated state trajectory) till a new state-input trajectory pair is received. We refer the reader to, e.g., \cite{findeisen2009stabilizing} for more insight on the topic. 

\subsection{Distributed control: a consensus algorithm}

In this section, we deviate from the previous example of vehicle control to illustrate the design of a distributed controller where, in contrast to what discussed above, the control input is computed as a function of the local state and of the state of the neighbouring systems according to a pre-defined network topology.

Consider a set of <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/c9b353093f15a8a991c5d28d0d066942.svg?invert_in_darkmode" align=middle width=45.00903pt height=22.38192pt/> single integrator systems 
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/7a78a53ed917914a7228fcc3d826c3af.svg?invert_in_darkmode" align=middle width=552.75pt height=16.376943pt/></p>
where <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/69bf836bbfaebdc81223c4735eabe92f.svg?invert_in_darkmode" align=middle width=59.889555pt height=24.56553pt/> and <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/2f4b65979b2ab55c89842f28e7f0c01c.svg?invert_in_darkmode" align=middle width=59.904735pt height=24.56553pt/> are the state and the input vectors at time <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/b3df72b25ba6b40f79f626256f13404a.svg?invert_in_darkmode" align=middle width=40.225845pt height=20.83059pt/>, respectively, and <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/e714a3139958da04b41e3e607a544455.svg?invert_in_darkmode" align=middle width=15.888015pt height=14.10255pt/> is the initial state vector <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/c49cde9cc85adba5bbc92ca668fd2b6f.svg?invert_in_darkmode" align=middle width=35.386065pt height=24.56553pt/> evaluated at the initial time <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/033bf1b4f4ab7245ca54b8b6d664d5e4.svg?invert_in_darkmode" align=middle width=45.16644pt height=22.56408pt/> of the generic agent <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/77a3b857d53fb44e33b53e4c8b68351a.svg?invert_in_darkmode" align=middle width=5.642109pt height=21.60213pt/>. The agents communicate among each others according to a communication graph <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/1c4d60841c9cf6bdd602357741556631.svg?invert_in_darkmode" align=middle width=78.663255pt height=24.56553pt/>, where the vertex set <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/76105ebc974ce8a02de91bcaf0d6d25f.svg?invert_in_darkmode" align=middle width=11.38203pt height=22.38192pt/> collects all the indexes of the agents, that is, <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/51c4ade1e6af2f4d8249fab321fa89ee.svg?invert_in_darkmode" align=middle width=92.88477pt height=22.38192pt/>, and the edge set <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/f9cdf81f147a475aee46d026c680846c.svg?invert_in_darkmode" align=middle width=74.79615pt height=22.38192pt/> is such that <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/4fc3276988a7ff7c8edcd2c6396f636b.svg?invert_in_darkmode" align=middle width=63.51213pt height=24.56553pt/> if and only if the system <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/77a3b857d53fb44e33b53e4c8b68351a.svg?invert_in_darkmode" align=middle width=5.642109pt height=21.60213pt/> can access <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/30d4d82f10a5ad82e7b07de8c55e359a.svg?invert_in_darkmode" align=middle width=42.413085pt height=29.12679pt/>. Therefore, agent <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/77a3b857d53fb44e33b53e4c8b68351a.svg?invert_in_darkmode" align=middle width=5.642109pt height=21.60213pt/> can access from their neighborhoods <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/16dde453a8a616487ff5e2d306fdd1ae.svg?invert_in_darkmode" align=middle width=171.275445pt height=24.56553pt/> at time <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/4f4f4e395762a3af4575de74c019ebb5.svg?invert_in_darkmode" align=middle width=5.913963pt height=20.1465pt/> the coordination vectors  <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/5ebd1f8ba2c174f336526f016c4a7b4f.svg?invert_in_darkmode" align=middle width=183.782445pt height=29.12679pt/>. Moreover, let <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/ccb3b8efd3cde89b19f11bf6feb490bb.svg?invert_in_darkmode" align=middle width=68.03412pt height=24.56553pt/> be the adjacency matrix of the communication graph <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/68a463cbf8842017bbbab8ca879333c7.svg?invert_in_darkmode" align=middle width=10.713285pt height=22.38192pt/>, such that <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/ca63d0fc97af39b8250a3b110fa5ac9d.svg?invert_in_darkmode" align=middle width=50.29233pt height=21.10812pt/> for all <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/17af8c5be8258eaaadb49f300988ab09.svg?invert_in_darkmode" align=middle width=43.58574pt height=22.38192pt/> and <img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/6688925f3eda5d97bf5f6fb3d81b5420.svg?invert_in_darkmode" align=middle width=50.29233pt height=21.10812pt/> otherwise. Applying the consensus control law 
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/30c291b89b32f7bc08859006948aaaba.svg?invert_in_darkmode" align=middle width=440.55165pt height=39.127275pt/></p>
it is possible to show that the state of all the system will converge to a common value, i.e., to consensus  \cite{Reza2007}.
```matlab
N = 5;

for i = 1:N

v{i} = ICtSystem('StateEquation',...
@(t,x,u,varargin)u,'nx',1,'nu',1);    
v{i}.controller = ex05BasicConsensusController();
v{i}.initialCondition = i;

end

%% Network
% Adjacency matrix - loop
A            = zeros(N);
A(1,4)       = 1;
A(2:N,1:N-1) = eye(N-1);

s1 = IAgentSensor(@(t,agentId,agent,sensedAgentIds,sensedAgents)sensedAgent.x);

Ah = @(t) A;

%% VirtualArena
a = VirtualArena(v,...
'StoppingCriteria'  , @(t,as)t>10,...
'SensorsNetwork'    , {s1,Ah},...
'DiscretizationStep', 0.1,...
'PlottingStep'      , 1);

ret = a.run();
```
Specifically, this class specifies the measurements that the `agent`, with associated `agentId`, obtains from the neighbouring agents `sensedAgents`, with associated `sensedAgentIds`, obtained by VirtualArena from the, possibly time varying, adjacency matrix `A`. In the specific example of `ex5StateSensor`, each agent reads the state of the `detectableAgentsList`. Using this network measurement model, the controller \eqref{eq:consensus} is implemented as follows.
```matlab
classdef ex05BasicConsensusController < Controller
methods
function u = computeInput(obj,t,x,readings)
nNeigh = length(readings{1});
u = 0;
for i =1:nNeigh
u = u+(readings{1}{i} - x)/nNeigh;
end
end
end
end
```
In general, is possible to specify multiple network sensors. In the latter case,  \texttt{\footnotesize readings\{i\}\{j\}} refers to the j-<img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/0209623128bb346c3b5cef2c041d28b7.svg?invert_in_darkmode" align=middle width=15.34962pt height=22.74591pt/> measurament provided by the i-<img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/0209623128bb346c3b5cef2c041d28b7.svg?invert_in_darkmode" align=middle width=15.34962pt height=22.74591pt/> sensor.

\section{Key benefits and main features}\label{sec:benefit}
From the previous examples, it is possible to identify the main features and benefits of the proposed platform.

\subsection{Modularity, reusability, and maintenance}

VirtualArena promotes the development of a modular structure, where each component (e.g., a controller, a dynamical model...) is self-contained and satisfies pre-defined interfaces. Therefore, it is easy to switch among different components of the same kind, e.g., as illustrated in Section \ref{sec:ex3}, it is easy to switch among controllers. To give an example, each (state-less) controller in VirtualArena is defined as a subclass of the class `Controller` with abstract method `u = computeInput(t,x)`. In this case, each controller needs to implement the abstract method `u = computeInput(t,x)` which will be used by VirtualArena in the simulation phase, independently from the specific controller in hand. As a consequence, once an object is developed, the knowledge of its implementation is not necessary for its use. This abstraction allows to reuse/share components and easily maintain the architecture by simply replacing selected blocks. Fig. \ref{fig:classdiagram} shows a simplified class-diagram of the main classes of the toolbox. 

\subsection{Collaborative design and test}
When multiple developers are involved in the design of a control architecture, the formalization of common interfaces is required to connect the different components of the architecture. VirtualArena defines such interfaces, facilitating the assignment of the design of modules to different developers.

\subsection{Dissemination and extensibility}
The effort and the technological and theoretical background required for the implementation of advanced control strategies are two of the main obstacles inhibiting the dissemination of new research results in industrial applications. VirtualArena, allowing the development of modular controllers, promotes the sharing of new algorithms among the users and provides functions to install external modules from third-party repositories. For instance, the following code will automatically retrieve a code from a third party archive located at the remote `URL` and import specific folders to the MATLAB path for future use.

```matlab
vaInstall(URL)
```

\subsection{Implemented control-oriented common functions}
There are many common components/procedures required in the simulation/design of control systems (e.g., discretization procedures, linearization procedures, EKF design, data logging architecture, etc.). VirtualaArena comes with an increasing number of predefined functionalities devoted to making the development process easier. Next, a list the main functionalities currently available in VirtualArena are presented. We refer to the on-line/function documentation for the use of specific functions.

\subsubsection{Simulation}
The first set of features regards tools for the simulation of the control architecture.
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/a8344380c0dedf53859a11c8a2243fca.svg?invert_in_darkmode" align=middle width=674.7444pt height=303.87225pt/></p>

\subsubsection{System definition and manipulation}
VirtualArena contains a set of interfaces to define dynamical systems provides a set of methods to manipulate these systems.
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/bdf9c77b3fdf161052b62d91ca7dd5dd.svg?invert_in_darkmode" align=middle width=673.42605pt height=139.488855pt/></p>

\subsubsection{State estimation}
As illustrated in Section \ref{sec:ex2} and using the linearization methods described above, VirtualArena provides automatic generation and simulations of state observers.
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/8a43b03d09c0fa1ebd592c26cfb523bb.svg?invert_in_darkmode" align=middle width=673.4343pt height=119.762775pt/></p>

\subsubsection{Model predictive control}
A set of functionalities is provided to define and customize MPC controllers.
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/cadb49065134c87e30a2c9b07f8cacd0.svg?invert_in_darkmode" align=middle width=743.0676pt height=208.6326pt/></p>

\subsubsection{Motion control of underactuated vehicle}
On the modelling of dynamical system, VirtualArena provides some models of underactuated vehicle with different state-space representations.
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/0c9bccc853218a0ff71dd7c0b2f18a8d.svg?invert_in_darkmode" align=middle width=674.6982pt height=83.701695pt/></p>

\subsubsection{Supported controller}
Both state-less controller and controller with internal dynamical model are supported.
<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/3aea5b2faa5ed2dd74d6b6b6570edaee.svg?invert_in_darkmode" align=middle width=598.04745pt height=80.31078pt/></p>

<p align="center"><img src="https://rawgit.com/andreaalessandretti/VirtualArena/master/svgs/73a1086d5984e3d835481ea2ee79ade0.svg?invert_in_darkmode" align=middle width=698.2833pt height=110.809545pt/></p>

\section{Conclusion and Future Works}\label{sec:conclusion}
This paper introduced the VirtualArena toolbox highlighting main features, keys benefits, and illustrating by example the use of the toolbox for the simulation and single-agent/multi-agent systems and evaluation of control schemes via simulations.

Future works consider further extensions of the toolkit in the direction of multi-agent control and estimation as well as the production of interfaces for existing third-party toolbox of interest, such as, e.g., fast solver for MPC optimization problem.
\bibliographystyle{abbrv} 
\bibliography{mybib}
\end{document}
