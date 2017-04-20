Let $I$ be an inertial coordinate frame and $B$ be a body coordinate frame attached to an UAV. The pair $(p(t),R(t)) \in SE(2)$\footnote{\label{note1}For a given $n\in\mathbb{N}$, $SE(n)$ denotes the Cartesian product of $\mathbb{R}^n$ with the group $SO(n)$ of $n \times n$ rotation matrices and $se(n)$ denotes the Cartesian product of $\mathbb{R}^n$ with the space  $so(n)$ of $n \times n$ skew-symmetric matrices. } denote the configuration of the vehicle, position and orientation, where  $R(t)$ is the rotation matrix from  body to inertial coordinates. Now, let $(v(t),\Omega(\omega(t)))~\in~SE(2)$ %\footnotemark[\ref{note1}]
 be the twist that defines the velocity of the vehicle, linear and angular, where the matrix $\Omega(\omega(t))$ is the skew-symmetric matrix associated to the angular velocity $\omega(t)\in\mathbb{R}$, defined as
$
\Omega(\omega) := 
\begin{pmatrix}
0  &  -\omega \\
\omega &  0 
\end{pmatrix}\nonumber.
$
The kinematic model of the body frame satisfieddds
$$
\dot{p}(t) &= R(t)v(t),& \dot{R}(t) = R(t)\Omega(\omega(t)).
$$
The underactuation of the UAV is captured by choosing the control input 
\begin{align}
u(t) := \begin{pmatrix} v_{1}(t)& \omega(t)\end{pmatrix}' \label{eq:constraints},
\end{align}
\end{subequations}
with $
v(t)~=~\begin{pmatrix}~v_{1}(t)& 0\end{pmatrix}'~\in~\mathbb{R}^2, \nonumber
$
that only consists of forward and angular velocity. The planar model \eqref{eq:constraints} captures the under-actuated nature of a wide range of vehicles and is often used for the control of a set of UAV moving in at 2-D plane, i.e., relying on an on-board inner controller for altitude stabilization. 
