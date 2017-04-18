classdef EkbfFilter < CtSystem & StateObserver & NoInitDeinitObject
    %EkbfFilter Extended Kalman-Bucy Filter
    %
    % filter = EkbfFiler(sys, 'Property1',PropertyValue1,'Property2',PropertyValue2,...)
    % where sys is the CtSystem used to design the filter.
    % Properties
    %
    % InitialStateEstimate
    %
    % InitialCovarianceMatrix
    % StateNoiseMatrix
    % OutputNoiseMatrix
    %
    % ex:
    % filter = EkbfFilter(model,...
    %	'StateNoiseMatrix'  , Qobs,...
    %	'OutputNoiseMatrix' , Robs,...
    %	'InitialCondition' , [ 0.01*ones(nx,1);
    %                           10  *reshape(eye(nx),nx^2,1)]);
    %     
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
         % To be changed for special errors, e.g., error between angles
        function inn = innovationFnc(obj,z,y)
            inn = z-y;
        end
        
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
            
            %obj.f      = @(t,xP,StUz)obj.ekbfEquations(t,StUz(1:sys.nu),xP,StUz(sys.nu+1:end));
            
            if not(isa(obj.system,'LinearizedSystem'))
                
                disp('Warning: System should ba a ''LinearizedSystem''');
                %obj.system.computeLinearization();
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
            
            %if nargin(obj.system.h)==3 
                obj.inputDependentOutput = 1;
            %end
            
                
        end
        
        function xPDot = f(t,xP,StUz,varargin)
            u = StUz(1:obj.system.nu);
            xP = xP;
            z = StUz(obj.system.nu+1:end);
            
            nx = obj.system.nx;
            ny = obj.system.ny;
            
            x = xP(1:nx);
            
            P = reshape( xP(nx+1:nx+nx^2),nx,nx);
            
            A = obj.system.A(t,x,u);
            if nargin(obj.system.C)== 2
                C = obj.system.C(t,x);
            elseif nargin(obj.system.C)== 3
                C = obj.system.C(t,x,u);
            end
            
            Q = obj.Qekf;
            R = obj.Rekf;
            %f = obj.system.f;
            h = obj.system.h;
            
            K    = P*C'/R;
            
            %if nargin(h)==3 
                y = h(t,x,u);
            %else
            %    y = h(t,x);
            %end
            
            fx = f(t,x,u);
            inn  = obj.innovationFnc(z,y);
            xDot = fx + K*inn;
            PDot = A*P + P*A' + Q - K*R*K';
            
            xPDot               = zeros(nx+nx^2,1);
            xPDot(1:nx)         = xDot;
            xPDot(nx+1:nx+nx^2) = reshape(PDot,nx^2,1);
            
        end
        
        
    end
end