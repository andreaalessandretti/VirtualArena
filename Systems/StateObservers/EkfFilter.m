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