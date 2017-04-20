

This section introduces VirtualArena toolkit by illustrating all the steps for the implementation a common control architecture.

## Model and state feedback controller definition

Let $I$ be an inertial coordinate frame and $B$ be a body coordinate frame attached to an UAV. The pair $(p(t),R(t)) \in SE(2)$ denote the configuration of the vehicle, position and orientation, where  $R(t)$ is the rotation matrix from  body to inertial coordinates. Now, let $(v(t),\Omega(\omega(t)))~\in~SE(2)$ be the twist that defines the velocity of the vehicle, linear and angular, where the matrix $\Omega(\omega(t))$ is the skew-symmetric matrix associated to the angular velocity $\omega(t)\in\mathbb{R}$, defined as
$
\Omega(\omega) := 
\begin{pmatrix}
0  &  -\omega \\
\omega &  0 
\end{pmatrix}\nonumber.
$
The kinematic model of the body frame satisfies
\begin{subequations}\label{eq:constrained_model}
\begin{align}
\dot{p}(t) &= R(t)v(t),& \dot{R}(t) = R(t)\Omega(\omega(t)).\label{eq:model}
\end{align}
The underactuation of the UAV is captured by choosing the control input 
\begin{align}
u(t) := \begin{pmatrix} v_{1}(t)& \omega(t)\end{pmatrix}' \label{eq:constraints},
\end{align}
\end{subequations}
with $
v(t)~=~\begin{pmatrix}~v_{1}(t)& 0\end{pmatrix}'~\in~\mathbb{R}^2, \nonumber
$
that only consists of forward and angular velocity. The planar model \eqref{eq:constrained_model} captures the under-actuated nature of a wide range of vehicles and is often used for the control of a set of UAV moving in at 2-D plane, i.e., relying on an on-board inner controller for altitude stabilization. 


Consider the \emph{control point} that we wish to control
\begin{align}
c(t):=p(t) + R(t)\epsilon\label{eq:cc}
\end{align} that is a constant point in the body frame placed at a constant distance $\epsilon=(\epsilon_1,\epsilon_2)'\in\mathbb{R}^2$ from the center of rotation of the body frame ${B}$. For a desired trajectory $c_d:\mathbb{R}\to\mathbb{R}^2$ for the control point \eqref{eq:cc}, it is possible to show (see \cite{Alessandretti2013,Alessandretti2017}) that the control law
\begin{align}
u(t) &= \Delta^{-1}(R'\dot{c}_d - Ke)\label{eq:uinp}\\
e(t) &= R'(c(t) - c_d(t)) \label{eq:defe}
\end{align} with $\Delta =\begin{pmatrix}1,&-\epsilon_2\\0,&\epsilon_1\end{pmatrix}$ for any $\epsilon_1\neq 0$ stabilizes the origin of the tracking error space $e=0$. The following code is used to simulated the closed-loop \eqref{eq:constrained_model} with \eqref{eq:uinp} with associated plots displayed in Fig. \ref{fig:ex1}.

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

\begin{figure}
\begin{center}
\includegraphics[width=9cm]{figs/fig2.pdf}    % The printed column  
\caption{Plot of simulation in Section \ref{sec:ex1}. Closed-loop state trajectories for the system simulated from four different initial conditions.}  % width is 8.4 cm.
\label{fig:ex1} % Size the figures 
\end{center}      % accordingly.
\end{figure}

\subsection{ Output feedback and EKF design} \label{sec:ex2}

The previous example considers the state feedback case. This section assumes that the state of the vehicle is observed with the observation model $y(t) = p(t)$, i.e., that captures, for instance,  the case where only the position, and not the heading, is available for control design, e.g., a GPS measurement. An Extended Kalman Filter (EKF, see, e.g., \cite{Anderson2005}) is designed on the vehicle model to estimate the state of the system, and then the same state feedback controller of the previous section is adopted for the motion control.

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
The automatic generation of the EKF with initial covariance matrix $ P_0=10I_{3\times 3}$, state noise covariance matrix $Q =\diag(([0.1,0.1,\pi/4])/3)^2$, and output noise covariance matrix $R = \diag(([0.1,0.1])/3)^2$, where $D=\diag(v)$ denotes a diagonal matrix $D$ with $D_{ij}=v_i$ and $I_{n\times n}$ represents the identity matrix of size $n$, is obtained as follows.
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
\begin{figure}
\begin{center}
\includegraphics[width=9cm]{figs/fig3.pdf}    % The printed column  
\caption{Plot of simulation in Section \ref{sec:ex2}.Closed-loop state trajectories for the system with disturbance simulated from forty different initial conditions. Each simulation is associated with a different realization of the noise signal. The continuous line denotes the state of the system and the dashed line its estimate.} % width is 8.4 cm.
\label{fig:cost} % Size the figures 
\end{center}      % accordingly.
\end{figure}

\subsection{Model Predictive Control implementation}\label{sec:ex3}

Building on the previous examples, the controller `TrackingController_ECC13` is replaced with an Model Predictive Control (MPC) control law designed to drive the error vector \eqref{eq:defe} to the origin. Specifically, we consider the MPC tracking controller from \cite{Alessandretti2013} where the terminal cost and terminal set is re-designed as in \cite{Alessandretti2017} to obtain global convergence. The following definition of open-loop MPC problem is considered in the definition of the MPC controller.
\begin{subequations} 
\begin{definition}[Open-Loop MPC Problem]\label{eq:P} Given a pair $(k,z)~\in~\mathbb{R}_{\geq k_0} \times \mathcal{X}(k)$ and an integer horizon length $N>1$, the open-loop MPC optimization problem $\mathcal{P}(k,z)$ consists of finding the optimal control sequence $\bbaru^*:=\{u^*(k),u^*(k+1),\dots,u^*(k+N-1)\}$ that solves
\begin{align} 
J_N^*(k,z) &= \min_{\boldsymbol{\bar{u}}}   J_N(k,z,\boldsymbol{\bar{u}}) \nonumber\\
\text{s.t.} \quad & \bar{x}(i+1)=f(i,\bar{x}(i),\bar{u}(i),0),&  i \in \mathbb{Z}_{k:k+N-1}\nonumber\\
\quad& (\bar{x}(i),\bar{u}(i)) \in \mathcal{X}(i)\times \mathcal{U}(i),&  i \in \mathbb{Z}_{k:k+N-1}\nonumber\\
\quad& \bar{x}(k) = z,\nonumber\\
\quad& \bar{x}(k+N) \in \mathcal{X}_{aux}(k+N) \nonumber 
\end{align}
with, $\bbaru=\{\bar{u}(k),\bar{u}(k+1),\dots,\bar{u}(k+N-1)\}$ and
\begin{align}
J_N(k,z,\boldsymbol{\bar{u}}) &:= \sum_{i=k}^{k+N-1} l(i,\bar{x}(i),\bar{u}(i))\nonumber\\
&\qquad\qquad\qquad+ m(k+N,\bar{x}(k+N)),\label{eq:Jn}
\end{align}
%
where the \emph{finite horizon cost} $J_N(\cdot)$, which corresponds to the \emph{performance index} of the MPC controller, is composed of the \emph{stage cost} $l~:~\mathbb{Z}_{\geq k_0}~\times~\mathbb{R}^n~\times~\mathbb{R}^m~\rightarrow~\mathbb{R} $ and the \emph{terminal cost} $m~:~\mathbb{Z}_{\geq k_0} \times \mathbb{R}^n~\rightarrow~\mathbb{R}$, where the last is defined over the time-varying \emph{terminal set} $\mathcal{X}_{aux}:\mathbb{Z}_{\geq k_0}\rightrightarrows \mathbb{R}^n$.\hfill $\square$%\hfill $\square$
\end{definition}
\end{subequations}

In order to make explicit the dependence of the optimal solution to the parameters of the MPC open-loop optimization problem, the term $\bar{u}^*(i;\hat{k},\hat{x})$ denotes the $i$-th input of the optimal input sequence $\bbaru^*$ computed by solving $\mathcal{P}(\hat{k},\hat{x})$. 

A standard MPC controller is obtained by solving at every time step $k\geq k_0$ the open-loop MPC problem and applying the first input of the optimal input trajectory to the system as follows
\begin{align} 
u(k)=k_{MPC}(k,x(k)) := \bar{u}^*(k;k,x(k)) \label{eq:kmpc}.
\end{align}

Following \cite{Alessandretti2017}, choosing the performance index with
\begin{align}
l(k,x,u)=\|e(t)\|^2,&& m(k,x,u)=0.3333\|e(t)\|^3,
\end{align}
the input constraint set 
$$\mathcal{U}(t)=\left\{u: \begin{matrix}-1.1 \leq v_1 \leq 1.1\\-1.1 \leq \omega \leq 1.1\end{matrix}\right\}$$ and the state constraint sets $\mathcal{X}(t)=\mathcal{X}_{aux}(t)=\mathbb{R}^3$ satisfies the sufficient conditions for global convergence of the error vector to the origin. In VirtualArena, this MPC controller can be defined as follows:
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

\begin{figure}
\begin{center}
\includegraphics[width=9cm]{figs/fig4.pdf}    % The printed column  
\caption{Plot of simulation in Section \ref{sec:ex3}. Closed-loop state trajectories for the system with disturbance simulated from four different initial conditions, ten times each. Each simulation is associated with a different realization of the noise signal. The continuous line denotes the state of the system and the dashed line its estimate. } % width is 8.4 cm.
\label{fig:cost} % Size the figures 
\end{center}      % accordingly.
\end{figure}

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

\begin{figure}
\begin{center}
\includegraphics[width=9cm]{figs/fig5.pdf}    % The printed column  
\caption{Plot of simulation in Section \ref{sec:ex4}. The dashed line denote the state trajectory estimate and the circles denote the measurements received from the remote system.} % width is 8.4 cm.
\label{fig:plotpy} % Size the figures 
\end{center}      % accordingly.
\end{figure}

Fig. \ref{fig:plotpy} shows the associated closed-loop trajectory. Notice that, in contrast to the previous examples, the real state of the vehicle is not displayed, since not available, and only the measurements the state estimate is shown.

This section only presents an illustrative example of a basic NCS. Although, for the control of a real system with fast dynamics, delays, and dropouts in the communication link, it is opportune to design a robust NCS schemes. 

Model Predictive Control, with its ability to generate future state and input prediction, is particularly suitable for this kind of applications. In fact, instead of sending only the input associated with the current time, it is possible to send the whole state and input predicted trajectory to a ``smart'' actuator that will apply open-loop the input trajectory to the system (or track the associated state trajectory) till a new state-input trajectory pair is received. We refer the reader to, e.g., \cite{findeisen2009stabilizing} for more insight on the topic. 

\subsection{Distributed control: a consensus algorithm}

In this section, we deviate from the previous example of vehicle control to illustrate the design of a distributed controller where, in contrast to what discussed above, the control input is computed as a function of the local state and of the state of the neighbouring systems according to a pre-defined network topology.

Consider a set of $N>0$ single integrator systems 
\begin{align}
\dot{x}(t)=u(t)&,x(t_0)=x_0&,i=1,\dots,N.
\end{align}
where $x(t) \in \mathbb{R}$ and $u(t) \in \mathbb{R}$ are the state and the input vectors at time $t\geq t_0$, respectively, and $x_0$ is the initial state vector $x(t_0)$ evaluated at the initial time $t_0\in\mathbb{R}$ of the generic agent $i$. The agents communicate among each others according to a communication graph $\mathcal{G}:=(\mathcal{V},\mathcal{E})$, where the vertex set $\mathcal{V}$ collects all the indexes of the agents, that is, $\mathcal{V}={1,\dots,N}$, and the edge set $\mathcal{E}\subseteq \mathcal{V}\times \mathcal{V}$ is such that $(i,j)\in \mathcal{E}$ if and only if the system $i$ can access $\gamma^{[j]}(t)$. Therefore, agent $i$ can access from their neighborhoods $j \in\mathcal{N}:=\{j:(i,j)\in \mathcal{E}\}$ at time $t$ the coordination vectors  $x_{\mathcal{N}}(t):=\{x^{[j]}(t):j\in \mathcal{N}\}$. Moreover, let $A:=[a_{ij}]$ be the adjacency matrix of the communication graph $\mathcal{G}$, such that $a_{ij}=1$ for all $j\in\mathcal{N}$ and $a_{ij}=0$ otherwise. Applying the consensus control law 
\begin{align}
u(t)=\sum_{j\in\mathcal{N}}(x^{[j]}(t)-x(t)) \label{eq:consensus}
\end{align}
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
In general, is possible to specify multiple network sensors. In the latter case,  \texttt{\footnotesize readings\{i\}\{j\}} refers to the j-$th$ measurament provided by the i-$th$ sensor.

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

###Simulation
The first set of features regards tools for the simulation of the control architecture.
- Time discretization methods. VirtualArena comes with different time discretization methods build-in such as `EulerForward`, `RK4`, `MatlabSolver`, that build on the `ode` solvers of MATLAB. Each discretization methods is defined as a subclass of `Integrator`. Other, and possible custom, discretisation methods can be created extending the latter class. 
- Logging management system. Easy-to-use logging management system with custom logging modules subclass of the abstract class `Log`. Implemented logs are `TimeLog`, `StateLog`, `ObserverStateLog`, `MeasurementsLog`, `InputLog`, `ControllerStateLog`, `ILog`.
- Multiple simulations. Easy-to-use system to run multiple simulations with different initial conditions and initial settings (e.g., different initial conditions, different controller parameters, etc.) implementing the abstract class `MultiRun`.
- Multi-agent systems. Simulation of a communication/sensor network for multi-agent systems implementing the abstract class `Sensor`: `RangeFinder`, `AgentSensor`, `IAgentSensor`.
- Simultaneous simulation of a network of multiple vehicles.

###System definition and manipulation
VirtualArena contains a set of interfaces to define dynamical systems provides a set of methods to manipulate these systems.
 
- Definition. Each system is defined as subclass of `GenericSystem`. The two main implementations of the dynamical systems are `CtSystem` and `DtSystem`.
- A `CtSystem` can be automatically discretized to create a `DtSystem`.
- A `DynamicalSystem`, either `CtSystem` or `DtSystem`, can be automatically linearized, where the computation of the jacobian matrices required for the linearization are automatically computed via Symbolic MATLAB or via sampling.

###State estimation
As illustrated in Section \ref{sec:ex2} and using the linearization methods described above, VirtualArena provides automatic generation and simulations of state observers.

- Automatic generation of Extended Kalman Filter for discrete-time dynamical systems in `EkfFilter`.
- Automatic generation of Extended Kalman-Bucy Filter for continuous-time dynamical systems in `EkbfFilter`
- Support for custom observers.


###Model predictive control
A set of functionalities is provided to define and customize MPC controllers.

- Definition of continuous-time and discrete-time MPC optimization problem, `CtMpcOp` and `DtMpcOp` respectively, subclasses of `MpcOp`.
- Definition of abstract class for MPC solver `MpcOpSolver` and warm-start strategies `WarmStart`.
- Implementation of a discrete-time MPC solver using `fmincon` in `FminconMpcOpSolver` (subclass of `MpcOpSolver`).
- Implementation of different warm-start strategies `ZerosWarmStart`, `ShiftAndAppendZeroWarmStart`, `AuxLawWarmStart`, `ShiftAndHoldWarmStart`,  and `ShiftAndAppendAuxLawWarmStart`.


###Motion control of underactuated vehicle
On the modelling of dynamical system, VirtualArena provides some models of underactuated vehicle with different state-space representations.

- Generic dynamical model representing `Unicycle` and the 3-D version `UAV` subclasses of `UnderactuatedVehicle`.
- Different representations of attitude using quaternions and rotation matrices available in `UnderactuatedVehicle`.


###Supported controller
Both state-less controller and controller with internal dynamical model are supported.

- Controller without internal dynamical model subclass of `Controller`.
- Discrete time controller with internal dynamical model, subclass of  `DtSytem`
- Continuous time controller with internal dynamical model, subclass of  `CtSytem`


\begin{figure}
\begin{center}
\includegraphics[width=8cm]{figs/fig6.pdf}    % The printed column  
\caption{Simplified class diagram of the main components of VirtualArena. The solid arrow denotes inheritance/implementation, where a class A points to a class B if A is a subclass of B and implements its abstract methods in the case of abstract class B (denoted in italic). The dashed arrow denotes dependency, where if A points to B then A ``uses'' B to function.} % width is 8.4 cm.
\label{fig:classdiagram} % Size the figures 
\end{center}      % accordingly.
\end{figure}

\section{Conclusion and Future Works}\label{sec:conclusion}
This paper introduced the VirtualArena toolbox highlighting main features, keys benefits, and illustrating by example the use of the toolbox for the simulation and single-agent/multi-agent systems and evaluation of control schemes via simulations.

Future works consider further extensions of the toolkit in the direction of multi-agent control and estimation as well as the production of interfaces for existing third-party toolbox of interest, such as, e.g., fast solver for MPC optimization problem.
