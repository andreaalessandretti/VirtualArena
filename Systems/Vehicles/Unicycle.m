
classdef Unicycle < UnderactuatedVehicle
%Unicycle Continuous time model of a Unicycle 
%
% v1 = Unicycle()
%
% State vector: x = [p;theta]
% 
% - p     : 2-d vector denoting the position of the vehicle.
% - theta : heading of the vehicle in rad.
%
% StateEquation :
%
% xDot = [u(1)*cos(x(3));
%         u(1)*sin(x(3));
%         u(2)];
%
% See also Vehicle, UAV
%


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


    properties
    
    end
    
    methods
 
    end
    
    methods
        function obj =  Unicycle(varargin)

             obj = obj@UnderactuatedVehicle(...
                'PositionSpaceDimension',2,...
                'Lvk',[1;0],'Lwk',1,'nu',2,...
                'OutputEquation',@(x,u)x(1:2),...
                varargin{:});
          
        end
        
          
    end
    
     methods(Static)
      
               
         function xDot = stateEquation(x,u)
            
            R = [cos(x(3)),-sin(x(3));
                 sin(x(3)), cos(x(3))];

            xDot = [R*[u(1);0];u(2)];
            
         end
         
          function ret = R(x)
               ret = [cos(x(3)),-sin(x(3));
                      sin(x(3)), cos(x(3))];
          end
    end
    
end