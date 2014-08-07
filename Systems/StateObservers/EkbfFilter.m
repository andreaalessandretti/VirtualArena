classdef EkbfFilter < CtSystem & NoInitDeinitObject
    %EkbfFilter Extended Kalman-Bucy Filter
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
    
    
    methods
        
        function obj = EkbfFilter(sys,varargin)
            
            if not(isa(sys,'CtSystem'))
                error('EkbfFilter supports CtSystem only.')
            end
            
            obj = obj@CtSystem(...
                'nx',sys.nx+sys.nx^2,...
                'nu',sys.ny,...
                'ny',sys.nx,...
                'OutputEquation',@(t,xP)xP(1:sys.nx),...
                varargin{:}...
                );
            
            obj.system = sys;
            
            obj.f      = @(t,xP,StUz)obj.ekbfEquations(t,StUz(1:sys.nu),xP,StUz(sys.nu+1:end));
            
            if isempty(obj.system.A) || isempty(obj.system.B) || isempty(obj.system.p) || isempty(obj.system.q) || isempty(obj.system.C) || isempty(obj.system.D)
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
        
        
        function  xPDot = ekbfEquations(obj,t,u,xP,z) % Prediction-Update Form from i-1 to i
            
            nx = obj.system.nx;
            ny = obj.system.ny;
            
            x = xP(1:nx);
            
            P = reshape( xP(nx+1:nx+nx^2),nx,nx);
            
            A = obj.system.A(x,u);
            C = obj.system.C(x,u);
            Q = obj.Qekf;
            R = obj.Rekf;
            f = obj.system.f;
            h = obj.system.h;
            
            K    = P*C'/R;
            
            if nargin(h)==3
                y = h(t,x,zeros(ny,1));
            else
                y = h(t,x);
            end
            
            if nargin(f)==4
                fx = f(t,x,u,zeros(nx,1));
            else
                fx = f(t,x,u);
            end
            
            inn  = (z-y);
            xDot = fx + K*inn;
            PDot = A*P + P*A' + Q - K*R*K';
            
            xPDot               = zeros(nx+nx^2,1);
            xPDot(1:nx)         = xDot;
            xPDot(nx+1:nx+nx^2) = reshape(PDot,nx^2,1);
            
        end
        
        
    end
end