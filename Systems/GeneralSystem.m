
classdef GeneralSystem < handle
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
        
        function computeLinearization(obj,varargin)
            
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