
classdef UniGoToPoint < Controller
    %UniGoToPoint go-to-position Controller for Unicycle models
    %   Given the state of a Unicycle vehicle, this Controller computes the
    %   input (i.e., linear and angular velocity) to steer the Unicycle to the
    %   point defined in the constructor
    %
    % UniGoToPoint Properties:
    %
    %   point                 - Desired position in 2-d
    %
    % UniGoToPoint Methods:
    %
    %   UniGoToPoint (point)
    %
    % See also Controller, Unicycle
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
        
        point; % Destination point
        
    end
    
    methods
        
        
        function obj = UniGoToPoint(point)
            
            obj.point = point;
            
        end
        
        function u = computeInput(obj,x,t)
            
            R = [cos(x(3)),-sin(x(3));
                sin(x(3)), cos(x(3))]; % Body to inertia rotation matrix
            
            err = R'*(obj.point - x(1:2)); % Position error in body frame
            
            thetaErr = atan2(err(2),err(1)); % Angle error
            
            u = [norm(obj.point-x(1:2));
                thetaErr];
            
        end
        
        
    end
    
end