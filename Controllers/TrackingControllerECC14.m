

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
% Copyright (c) 2014, Andrea Alessandretti
% All rights reserved.
%
% e-mail: andrea.alessandretti [at] {epfl.ch, ist.utl.pt}
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% 
% 1. Redistributions of source code must retain the above copyright notice, this
%    list of conditions and the following disclaimer. 
% 2. Redistributions in binary form must reproduce the above copyright notice,
%    this list of conditions and the following disclaimer in the documentation
%    and/or other materials provided with the distribution.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
% ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
% 
% The views and conclusions contained in the software and documentation are those
% of the authors and should not be interpreted as representing official policies, 
% either expressed or implied, of the FreeBSD Project.
    
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