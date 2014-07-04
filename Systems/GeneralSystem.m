
classdef GeneralSystem < handle & InitDeinitObject
    %GeneralSystem
    %
    % x' = f(x,u,v)
    % y  = h(x,u,w)
    %
    % x is an nx-dimensional vector
    % u is an nu-dimensional vector
    % y is an ny-dimensional vector
    %
    % v zero mean gaussian state noise with covariance Q
    % w zero mean gaussian output noise with covariance R
    %
    % (x',x,u,y) = (x(k+1),x(k),u(k),y(k),v(t),w(t))     in the discrte time case.
    % (x',x,u,y) = (\dot{x}(t),x(t),u(t),y(t),v(t),w(t)) in the continuous time case.
    %
    % GeneralSystem properties:
    %
    % nx,nu,ny     - dimension of vectors x,u, and y respectively
    % f,h          - function handles of f(x,u)/f(x,u,w) and h(x,u)/h(x,u,w)
    % Q,R          - covariance matrices of state and output noise, respectively
    %
    % A,B,p,C,D,q  - function handles @(xbar,ubar) of the parameters of the
    %                linearized model
    %
    %  x(k+1)/dot(x) = A(xbar,ubar) x(k) + B(xbar,ubar) u(k) + p(xbar,ubar)
    %  y(k)          = C(xbar,ubar) x(k) + D(xbar,ubar) u(k) + q(xbar,ubar)
    %
    % initialConditions - initial condition/conditions of the system
    % stateObserver     - state observer (of class StateObserver)
    % controller        - system controller of class Controller, CtSystem, or
    %                     DtSystem
    % x                 - current state vector
    %
    % See also CtSystem, DtSystem
    
    
 
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
        
        nx % Dimension of the state space
        
        nu % Dimension of the input space
        
        ny % Dimension of the output space
        
        f  % State equation ( function handle @(x,u) )
        
        h  % Output equation ( function handle @(x,u) )
        
        % it is posible to specify different initial conditions for multiple
        % simulations. In this case initialConditions(:,i) containts the
        % ith initial condition to simulate
        initialConditions
        
        stateObserver
        
        controller;      %controller used to drive the vehicle
        x;               %current state vector
        y;               %current output vector
        
        %State and input transformation, used to solve some numerical
        %problems
        Ax = []; %xbar = Ax*x
        Au = []; %ubar = Au*u
        
        
        %% Linearization
        % x(k+1)/dot(x) = A x(k) + B u(k) + p
        % y(k)           = C x(k) + D u(k) + q
        A,B,p,C,D,q
        
        % Noise
        Q
        R
    end
    
    
    methods
        
        
        function obj = GeneralSystem (varargin)
            %
            %       sys = GeneralSystem(par1,val1,par2,val2,...)
            %  where the parameters are chosen among the following
            %
            %   'nx', 'nu', 'ny', 'InitialConditions', 'Q', 'R', 'Controller',
            %   'StateEquation', 'OutputEquation'  (See f and h, respectively, above)
            %   'LinearizationMatrices' (value: {A,B,p,C,D,q})
            
            %% Retrive parameters for superclass GeneralSystem
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'nx'
                            
                            obj.nx = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'nu'
                            
                            obj.nu = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                        case 'ny'
                            
                            obj.ny = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StateEquation'
                            
                            obj.f = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'OutputEquation'
                            
                            obj.h = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Q'
                            
                            obj.Q = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'R'
                            
                            obj.R = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'InitialConditions'
                            
                            obj.initialConditions = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StateObserver'
                            
                            obj.stateObserver = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'LinearizationMatrices'
                            
                            mats = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                            obj.A = mats{1};
                            obj.B = mats{2};
                            obj.p = mats{3};
                            obj.C = mats{4};
                            obj.D = mats{5};
                            obj.q = mats{6};
                            
                        case 'Controller'
                            
                            obj.controller = varargin{parameterPointer+1};
                            
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
        
        
        function useSymbolicEvaluation(obj)
        %%useSymbolicEvaluation evaluates the state and output equations
        %   with symbolic variables, simplify them, and replace them with the
        %   simplified version. 
        %   
        %   This can lead to a notable decrease of computation time for 
        %   the case of complex models.
        
            x = sym('x',[obj.nx,1]);
            x = sym(x,'real');
            
            u = sym('u',[obj.nu,1]);
            u = sym(u,'real');
            
            if isempty(obj.Q)
                noiseF = {};
            else
                w = sym('w',[length(obj.Q),1]);
                w = sym(w,'real');
                noiseF = {w};
            end
            
            if isempty(obj.R)
                noiseH = {};
            else
                v = sym('v',[length(obj.R),1]);
                v = sym(v,'real');
                noiseH = {v};
            end
            
            fprintf(getMessage('GeneralSystem:evaluation'));
            obj.f = matlabFunction(simplify( obj.f(x,u,noiseF{:}) ) ,'vars',{x,u,noiseF{:}});
            obj.h = matlabFunction(simplify( obj.h(x,u,noiseH{:}) ) ,'vars',{x,u,noiseH{:}});
            fprintf(getMessage('done'));
        end
        
        function computeLinearization(obj,varargin)
        %%computeLinearization 
        %   computeLinearization()
        %   Computes the parametric matrices A, B, p, C, D, and q, with 
        %   parameters (xbar,ubar), associated with the linearized system
        %    
        %   x(k+1)/dot(x) = A(xbar,ubar) x(k) + B(xbar,ubar) u(k) + p(xbar,ubar)
        %   y(k)          = C(xbar,ubar) x(k) + D(xbar,ubar) u(k) + q(xbar,ubar)
        %
        %   By default these matrices are computed using the Symbolic
        %   Toolbox of Matlab and stored in the object as function handles.
        %
        %   computeLinearization('Sampled')
        %   In this case the matrices are computed using via sampling of
        %   the function. This mode is advised when the computation of the
        %   symbolic jacobians are prohibitive.
        %
            if isempty(obj.Q)
                noiseF = {};
            else
                noiseF = {zeros(size(obj.Q,1),1)};
            end
            
            if isempty(obj.R)
                noiseH = {};
            else
                noiseH = {zeros(size(obj.R,1),1)};
            end
            
            
            if nargin>1 & ischar( varargin{1})
                
                switch varargin{1}
                    case 'Sampled'
                        
                        %% Compute linearizations
                        if isa(obj.f,'function_handle')
                            
                            fprintf(getMessage('GeneralSystem:LinearizingStateEquationS'));
                            
                            obj.A = @(xbar,ubar)jacobianSamples(@(x)obj.f(x,ubar,noiseF{:}),xbar);
                            
                            obj.B = @(xbar,ubar)jacobianSamples(@(u)obj.f(xbar,u,noiseF{:}),ubar);
                            
                            obj.p = @(x,u) (  obj.f(x,u,noiseF{:}) - jacobianSamples(@(z)obj.f(z,u,noiseF{:}),x)*x - jacobianSamples(@(z)obj.f(x,z,noiseF{:}),u)*u);
                            
                            fprintf(getMessage('done'));
                            
                        end
                        
                        if isa(obj.h, 'function_handle')
                            
                            fprintf(getMessage('GeneralSystem:LinearizingOutputEquationS'));
                            
                            obj.C = @(xbar,ubar)jacobianSamples(@(x)obj.h(x,ubar,noiseH{:}),xbar);
                            
                            obj.D = @(xbar,ubar)jacobianSamples(@(u)obj.h(xbar,u,noiseH{:}),ubar);
                            
                            obj.q = @(x,u) (  obj.h(x,u,noiseH{:}) - jacobianSamples(@(z)obj.h(z,u,noiseH{:}),x)*x - jacobianSamples(@(z)obj.h(x,z,noiseH{:}),u)*u);
                            
                            
                            fprintf(getMessage('done'));
                            
                        end
                        
                        
                    otherwise
                        
                        %% Compute linearizations
                        
                        x = sym('x',[obj.nx,1]);
                        x = sym(x,'real');
                        
                        u = sym('u',[obj.nu,1]);
                        u = sym(u,'real');
                        
                        
                        
                        if isa(obj.f,'function_handle')
                            
                            fprintf(getMessage('GeneralSystem:LinearizingStateEquation'));
                            
                            obj.A = matlabFunction(  jacobian(obj.f(x,u,noiseF{:}),x)  ,'vars',{x,u});
                            
                            obj.B = matlabFunction(  jacobian(obj.f(x,u,noiseF{:}),u)  ,'vars',{x,u});
                            
                            obj.p = matlabFunction(  obj.f(x,u,noiseF{:}) - jacobian(obj.f(x,u,noiseF{:}),x)*x - jacobian(obj.f(x,u,noiseF{:}),u)*u  ,'vars',{x,u});
                            
                            fprintf(getMessage('done'));
                            
                        end
                        
                        if isa(obj.h, 'function_handle')
                            
                            fprintf(getMessage('GeneralSystem:LinearizingOutputEquation'));
                            
                            obj.C = matlabFunction(  jacobian(obj.h(x,u,noiseH{:}),x)  ,'vars',{x,u});
                            
                            obj.D = matlabFunction(  jacobian(obj.h(x,u,noiseH{:}),u)  ,'vars',{x,u});
                            
                            obj.q = matlabFunction(  obj.h(x,u,noiseH{:}) - jacobian(obj.h(x,u,noiseH{:}),x)*x - jacobian(obj.h(x,u,noiseH{:}),u)*u  ,'vars',{x,u});
                            
                            fprintf(getMessage('done'));
                            
                        end
                        
                end
                
            else
                
                
                %% Compute linearizations
                
                x = sym('x',[obj.nx,1]);
                x = sym(x,'real');
                
                u = sym('u',[obj.nu,1]);
                u = sym(u,'real');
                
                
                
                if isa(obj.f,'function_handle')
                    
                    fprintf(getMessage('GeneralSystem:LinearizingStateEquation'));
                    
                    obj.A = matlabFunction(  jacobian(obj.f(x,u,noiseF{:}),x)  ,'vars',{x,u});
                    
                    obj.B = matlabFunction(  jacobian(obj.f(x,u,noiseF{:}),u)  ,'vars',{x,u});
                    
                    obj.p = matlabFunction(  obj.f(x,u,noiseF{:}) - jacobian(obj.f(x,u,noiseF{:}),x)*x - jacobian(obj.f(x,u,noiseF{:}),u)*u  ,'vars',{x,u});
                    
                    fprintf(getMessage('done'));
                    
                end
                
                if isa(obj.h, 'function_handle')
                    
                    fprintf(getMessage('GeneralSystem:LinearizingOutputEquation'));
                    
                    obj.C = matlabFunction(  jacobian(obj.h(x,u,noiseH{:}),x)  ,'vars',{x,u});
                    
                    obj.D = matlabFunction(  jacobian(obj.h(x,u,noiseH{:}),u)  ,'vars',{x,u});
                    
                    obj.q = matlabFunction(  obj.h(x,u,noiseH{:}) - jacobian(obj.h(x,u,noiseH{:}),x)*x - jacobian(obj.h(x,u,noiseH{:}),u)*u  ,'vars',{x,u});
                    
                    fprintf(getMessage('done'));
                    
                end
            end
        end
        
        function stateInputTransformation(obj,Ax,Au)
            
            fOld = obj.f;
            obj.f = @(x,u) Ax*fOld(Ax\x,Au\u);
            hOld = obj.h;
            obj.h = @(x,u) hOld(Ax\x,Au\u);
            
            obj.Ax = Ax;
            obj.Au = Au;
        end
        
        function params = getParameters(obj)
            
            params = {...
                'nx', obj.nx, ...
                'nu', obj.nu, ...
                'ny', obj.ny,...
                'StateEquation', obj.f, ...
                'OutputEquation',obj.h, ...
                'Q',obj.Q,...
                'R',obj.R
                ...'LinearizationMatrices',{obj.A,obj.B,obj.p,obj.C,obj.D,obj.q}...
                };
        end
        
        
        
    end
end