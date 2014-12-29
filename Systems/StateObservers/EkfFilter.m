classdef EkfFilter < DtSystem & StateObserver
    %EkfFilter Extended Kalman Filter
    %
    % filter = EkfFiler(sys, 'Property1',PropertyValue1,'Property2',PropertyValue2,...)
    %
    % where sys is the CtSystem used to design the filter.
    %
    % Properties
    %
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
        
        inputDependentOutput = 0;
        
    end

    
    methods
        
        function obj = EkfFilter(sys,varargin)
            
            if not(isa(sys,'DtSystem'))
                error('EkfFilter is only for DtSystem.')
            end
            
            obj = obj@DtSystem(...
                'nx',sys.nx+sys.nx^2,...
                'nu',sys.ny,...
                'ny',sys.nx,...
                'OutputEquation',@(t,xP)xP(1:sys.nx),varargin{:});
            
            obj.system = sys;
            obj.f      = @(t,xP,StUz)obj.prediction(t,StUz(1:sys.nu),xP);
            
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
            
            if nargin(sys.h) == 3 % The output is input dependent
                obj.inputDependentOutput = 1;
            end
            
            
        end
        
        
        function  xNext = prediction(obj,t,u,xP) 
            
            sysnx = obj.system.nx;
            xHat  = xP(1:sysnx);
            P     = reshape( xP(sysnx+1:sysnx+sysnx^2),sysnx,sysnx);
            
            A     = obj.system.A(t,xHat,u);
            Q     = obj.Qekf;
            
            
            xHat = obj.system.f(t,xHat,u);
            P    = A*P*A' + Q;
            
            xNext                        = zeros(sysnx+sysnx^2,1);
            xNext(1:sysnx)               = xHat;
            xNext(sysnx+1:sysnx+sysnx^2) = reshape(P,sysnx^2,1);
            
        end
        
        
        function newXObs = preInputUpdate(obj,t,xP,z)
            
            newXObs = xP;
            
            if not(obj.inputDependentOutput)
                
                sysnx   = obj.system.nx;
                sysnu   = obj.system.nu;
                
                R       = obj.Rekf;
                
                xHat    = xP(1:sysnx);
                P       = reshape( xP(sysnx+1:sysnx+sysnx^2),sysnx,sysnx);
        
                inn = z-obj.system.h(t,xHat);

                % Since the output is not input dependent we can set an
                % arbitrary value of u
                C    = obj.system.C(t,xHat,zeros(sysnu,1)); 
                
                S    = C*P*C' + R;
                K    = P*C'/S;
                
                xHat = xHat + K*inn;
                P    = (eye(length(xHat)) - K*C)*P;
                
                newXObs                        = zeros(sysnx+sysnx^2,1);
                newXObs(1:sysnx)               = xHat;
                newXObs(sysnx+1:sysnx+sysnx^2) = reshape(P,sysnx^2,1);
            end
            
        end
        
        function  newXObs = postInputUpdate(obj,t,xP,z,u)
            
            newXObs = xP;
            R       = obj.Rekf;
            
            if obj.inputDependentOutput
                
                sysnx   = obj.system.nx;
                xHat    = xP(1:sysnx);
                P       = reshape( xP(sysnx+1:sysnx+sysnx^2),sysnx,sysnx);

                inn  = z-obj.system.h(t,xHat,u);

                C    = obj.system.C(t,xHat,u) 
                
                S    = C*P*C' + R;
                K    = P*C'/S;
               
                xHat = xHat + K*inn;
                P    = (eye(length(xHat)) - K*C)*P;
                
                newXObs                        = zeros(sysnx+sysnx^2,1);
                newXObs(1:sysnx)               = xHat;
                newXObs(sysnx+1:sysnx+sysnx^2) = reshape(P,sysnx^2,1);
                
            end
        end
        
        
        
    end
end