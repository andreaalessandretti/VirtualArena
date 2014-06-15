

classdef TrackingControllerECC14 < Controller
    
    %TrackingController is a tracking algorithm for Underactuated Vehicles.
    %   Given a vehicle of of the class UnderactuatedVehicle computes a
    %   control law that steers the position of the vehicle to an desired
    %   trajectory with maximum tracking  error of |epsilon|, of a given
    %   vector epsilon of arbitrary small module.
    %
    % TrackingController Properties:
    %
    % vehicle  - vehicles to control
    % epsilon  - tracking accuracy
    % pd       - @(t), desired position as function of time
    % dotPd    - @(t), time derivative of the desired position as function of time
    % Ke       - Lyapunov decrease gain for position error
    %
    % TrackingController Methods:
    %
    % TrackingControllerECC14 - help TrackingControllerECC14.TrackingControllerECC14
    %
    %   demo: exTrackingAlgorithmUAV.m
    %
    %   For more info we refer to:
    %   -------------------------------------------------------------------
    %   Alessandretti, A., Aguiar, A. P., & Jones, C. N. (2013).
    %
    %   Trajectory-tracking and path-following controllers for constrained
    %   underactuated vehicles using Model Predictive Control.
    %
    %   In European Control Conference (ECC), 2013
    %   -------------------------------------------------------------------
    %
    % See also Controller, UnderactuatedVehicle
    
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
        vehicle
        epsilon
        pd
        dotPd
        Ke
        Kxi
        
    end
    
    
    methods
        
        
        function obj = TrackingControllerECC14(varargin)
            % TrackingControllerECC14 is the costructor
            %
            %          va = TrackingControllerECC14(par1,val1,par2,val2,...)
            %
            %	where the parameters are chosen among the followings
            %
            %	'Vehicle', 'Epsilon', 'pd', 'dotPd', 'Ke'
            %
            %	see the descriptions of the associated properties.
            
            obj = obj@Controller();
            
            
            %% Retrive parameters for superclass GeneralSystem
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'Vehicle'
                            
                            obj.vehicle = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Epsilon'
                            
                            obj.epsilon = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                        case 'pd'
                            
                            obj.pd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'dotPd'
                            
                            obj.dotPd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Ke'
                            
                            obj.Ke = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
                if not(isempty(obj.vehicle.Lvd)) || not(isempty(obj.vehicle.Lwd)) || not(isempty(obj.vehicle.mw))|| not(isempty(obj.vehicle.mw))
                    error(getMessage('TrackingControllerECC14:wrongvehicle'));
                end
                
            end
            
            
            
        end
        
        
        function u = computeInput(obj,x,t)
            
            switch obj.vehicle.n
                case 2 % 2D case
                    Omega = @(omega) [0,-omega;omega,0];
                    OmegaEpsilon = @(x)[x(2);-x(1)];
                    
                case 3 % 3D case
                    Omega = @(omega) [ 0       ,-omega(3) ,  omega(2);
                        omega(3), 0        , -omega(1);
                        -omega(2), omega(1) ,  0];
                    OmegaEpsilon = Omega ;
            end
            
            
            
            R = obj.vehicle.getR(x);
            
            nuk = obj.vehicle.nvk+obj.vehicle.nwk;
            nd  = obj.vehicle.nvd+obj.vehicle.nwd;
            
            pd    = obj.pd(t);
            dotPd = obj.dotPd(t);
            epsilon = obj.epsilon;
            Ke = obj.Ke;
            
            Lvk = obj.vehicle.Lvk;
            Lwk = obj.vehicle.Lwk;
            
            
            p = obj.vehicle.getPosition(x);
            
            E = [];
            if not(isempty(Lvk))
                E = Lvk;
            end
            if not(isempty(Lwk))
                E = [E,OmegaEpsilon(epsilon)*Lwk];
            end
            
            
            if not(rank(E)==size(E,1))
                error('System not controllable');
            end
            
            
            PinvE = E'/(E*E');
            e = R'*(p-pd)-epsilon;
            
            u = [];
            
            r= -R'*dotPd ;
            
            
            Ekstar = PinvE(1:nuk,:);
            u = Ekstar*(-Ke*e -r);
            
            
        end
        
        
    end
    
end