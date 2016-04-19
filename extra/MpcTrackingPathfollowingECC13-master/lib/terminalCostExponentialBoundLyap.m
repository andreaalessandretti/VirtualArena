% cost = terminalCostExponentialBoundLyap(x,lc,a)
%
% - Lyapunov constants lc =  [al,k1,k2,k3]
%
%   s.t.
%
%   k1*x^al<=  V(x)  <=k2*x^al
%            dotV(x) <=k3*x^al
%
% - l bounds constants a  =  [a1,a2,...,am]
%
%   l(x,k_f(x))<= \sum_{i=1}^{length(a)} ai ||x||^i
%


% This file is part of ADDMPC.
%
% ADDMPC - Analysis, Design, and Deployment of Model Predictive Control laws.
% Copyright (C) 2012-13 Andrea Alessandretti
%
% andrea.alessandretti@{ist.utl.pt, epfl.ch}
% Automatic Control Laboratory, EPFL, Lausanne, Switzerland.
% Institute System and Robotics, IST, Lisbon, Portugal.
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
function cost = terminalCostExponentialBoundLyap(x,lc,a)
cost = 0;
al = lc(1);
kl = lc(2:end);
for i = 1:length(a)
    cost = cost + a(i)*(kl(2)/kl(1))^(i/al)*(al*kl(2)/(i*kl(3)))*(sqrt(x'*x))^i;
end
end