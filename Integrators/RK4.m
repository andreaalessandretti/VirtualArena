%RK4 is the implementation of the Runge-Kutta method RK4
%
%   See also Integrator, EulerForward


% This file is part of VirtualArena.
%
% Copyright (C) 2012-14 Andrea Alessandretti
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


classdef RK4 < Integrator
    
    
    properties
    end
    
    methods (Static)
        function xkp1 = integrate(f,xk,h)
            
            
            k1 = f(xk);
            k2 = f(xk + 0.5*h*k1);
            k3 = f(xk + 0.5*h*k2);
            k4 = f(xk + h*k3);
            
            xkp1 = xk +(1/6)*h*(k1+2*k2+2*k3+k4);
        end
    end
    
end

