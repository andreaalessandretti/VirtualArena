classdef EkfFilter < DtSystem
    %EkfFilter Extended Kalman Filter
    %
    % filter = EkfFiler('Property1',PropertyValue1,'Property2',PropertyValue2,...)
    %
    % Properties
    %
    % System                     - Supported systems : DtSystem
    % InitialStateEstimate
    %
    % InitialCovarianceMatrix
    % StateNoiseMatrix
    % OutputNoiseMatrix
    %
    % demo: exStateObserver.m
    
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
        
        Qekf
        
        Rekf
        
        system
    end
    
    properties (SetAccess = private, GetAccess = private)
        
        
    end
    
    methods
        
        function obj = EkfFilter(sys,varargin)
            
            if not(isa(sys,'DtSystem'))
                error('EkfFilter supports DtSystem only.')
            end
            
            obj = obj@DtSystem(...
                'nx',sys.nx+sys.nx^2,...
                'nu',sys.ny,...
                'ny',sys.nx,...
                'OutputEquation',@(xP)xP(1:sys.nx),varargin{:});
            
            obj.system = sys;
            obj.f=@(xP,StUz)obj.ekfEquations(StUz(1:sys.nu),xP,StUz(sys.nu+1:end));
            
            if isempty(obj.system.A) || isempty(obj.system.B) || isempty(obj.system.p) || isempty(obj.system.q) || isempty(obj.system.C) ||isempty(obj.system.D)
                disp('Warning: Linearization not found, computing linearization - ');
                obj.system.computeLinearization();
            end
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'StateNoiseMatrix'
                            
                            obj.Qekf = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'OutputNoiseMatrix'
                            
                            obj.Rekf = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
        end
        
        
        function  xNext = ekfEquations(obj,u,xP,z) % Prediction-Update Form from i-1 to i
            
            
            
            sysnx = obj.system.nx;
            
            xim1_im1 = xP(1:sysnx);
            
            Pim1_im1 = reshape( xP(sysnx+1:sysnx+sysnx^2),sysnx,sysnx);
            
            [xi_i,Pi_i] = obj.getUpdatedEstimate(xim1_im1,Pim1_im1,z,u);
            
            
            %% Update Trajectories
            xNext = zeros(sysnx+sysnx^2,1);
            xNext(1:sysnx) = xi_i;
            xNext(sysnx+1:sysnx+sysnx^2) = reshape(Pi_i,sysnx^2,1);
            
        end
        
        function [xi_i,Pi_i] = getUpdatedEstimate(obj,xim1_im1,Pim1_im1,z,u)
            
            
            A = obj.system.A(xim1_im1,u);
            
            Q = obj.Qekf;
            
            R = obj.Rekf;
            
            %% Filter Equation
            
            % Prediction
            if isempty(obj.system.Q)
                xi_im1 = obj.system.f(xim1_im1,u);
            else
                xi_im1 = obj.system.f(xim1_im1,u,zeros(size(obj.system.Q,1),1) );
            end
            
            Pi_im1 = A*Pim1_im1*A' + Q;
            
            % Update
            
            if isempty(obj.system.R)
                inn = z-obj.system.h(xi_im1,u);
            else
                inn = z-obj.system.h(xi_im1,u,zeros(size(obj.system.R,1),1) );
            end
            
            
            
            C = obj.system.C(xi_im1,u);
            
            S = C*Pi_im1*C' + R;
            
            K = Pi_im1*C'*inv(S);
            
            xi_i = xi_im1 + K*inn;
            
            Pi_i = (eye(length(xi_im1)) - K*C)*Pi_im1;
            
        end
        
    end
end