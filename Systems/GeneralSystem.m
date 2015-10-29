classdef GeneralSystem < handle & InitDeinitObject
    %GeneralSystem
    %
    % x' = f(t,x,u)
    % y  = h(t,x)/h(t,x,u)
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
    %
    % A,B,p,C,D,q  - function handles @(tbar,xbar,ubar) of the parameters 
    %                of the linearized model
    %
    %  x(k+1)/dot(x) = A(tbar,xbar,ubar) x(k) + B(tbar,xbar,ubar) u(k) + p(t,tbar,xbar,ubar)
    %  y(k)          = C(tbar,xbar,ubar) x(k) + D(tbar,xbar,ubar) u(k) + q(t,tbar,xbar,ubar)
    %
    % initialConditions - initial condition/conditions of the system
    % stateObserver     - state observer (of class StateObserver)
    % controller        - system controller of class Controller, CtSystem, or
    %                     DtSystem
    % x                 - current state vector
    %
    %
    %  sys = obj@GeneralSystem(par1,val1,par2,val2,...) % only by a subclass
    %
    %  where the parameters are chosen among the following
    %
    %   'nx', 'nu', 'ny', 'InitialCondition', 'Controller',
    %   'StateEquation', 'OutputEquation'  (See f and h, respectively, above)
    %   'LinearizationMatrices' (value: {A,B,p,C,D,q})
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
        
        f  % State equation ( function handle @(t,x,u) )
        
        h  % Output equation ( function handle @(t,x,u) )
        
        % it is posible to specify different initial conditions for multiple
        % simulations. In this case initialConditions(:,i) containts the
        % ith initial condition to simulate
        initialConditions
        
        stateObserver
        
        controller;      %controller used to drive the vehicle
        x;               %current state vector
        y;               %current output vector
        
        %% Linearization
        % x(k+1)/dot(x) = A x(k) + B u(k) + p
        % y(k)           = C x(k) + D u(k) + q
        A,B,p,C,D,q
        
        
        % changeOfCoordinate - Variable of the change of choordinates
        % x' = A x+b
        % u' = C u+d
        % t' = e t+f
        
        cA, cb, cC, cd, ce,cf
        
        %% Descriptors
        name
        
        stateName
        stateUnit
        state2d
        state3d
        
        inputName
        inputUnit
        input2d
        input3d
        
        outputName
        outputUnit
        output2d
        output3d
        
    end
    
   
    
    methods
        
        function obj = GeneralSystem (varargin)
            %
            %       sys = GeneralSystem(par1,val1,par2,val2,...)
            %  where the parameters are chosen among the following
            %
            %   'nx', 'nu', 'ny', 'InitialCondition', 'Q', 'R', 'Controller',
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
                            
                        case 'InitialCondition'
                            
                            obj.initialConditions = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StateObserver'
                            
                            obj.stateObserver = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Name'
                            
                            obj.name = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StateName'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.stateName = varargin{parameterPointer+1};
                            else
                                obj.stateName = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StateUnit'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.stateUnit = varargin{parameterPointer+1};
                            else
                                obj.stateUnit = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'State2d'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.state2d = varargin{parameterPointer+1};
                            else
                                obj.state2d = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'State3d'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.state3d = varargin{parameterPointer+1};
                            else
                                obj.state3d = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'InputName'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.inputName = varargin{parameterPointer+1};
                            else
                                obj.inputName = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'InputUnit'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.inputUnit = varargin{parameterPointer+1};
                            else
                                obj.inputUnit = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Input2d'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.input2d = varargin{parameterPointer+1};
                            else
                                obj.input2d = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Input3d'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.input3d = varargin{parameterPointer+1};
                            else
                                obj.input3d = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'OutputName'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.outputName = varargin{parameterPointer+1};
                            else
                                obj.outputName = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'OutputUnit'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.outputUnit = varargin{parameterPointer+1};
                            else
                                obj.outputUnit = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Output2d'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.output2d = varargin{parameterPointer+1};
                            else
                                obj.output2d = {varargin{parameterPointer+1}};
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Output3d'
                            
                            if iscell(varargin{parameterPointer+1})
                                obj.output3d = varargin{parameterPointer+1};
                            else
                                obj.output3d = {varargin{parameterPointer+1}};
                            end
                            
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
            
            t = sym('t',[1,1]);
            t = sym(t,'real');
            
            fprintf(getMessage('GeneralSystem:evaluation'));
            
            if not(isempty(obj.f))
                obj.f = matlabFunction(simplify( obj.f(t,x,u) ) ,'vars',{t,x,u});
            end
            
            if not(isempty(obj.h))
                obj.h = matlabFunction(simplify( obj.h(t,x,u) ) ,'vars',{t,x,u});
            end
            
            fprintf(getMessage('done'));
        end
        
        
        
        function computeLinearization(obj,varargin)
            %%computeLinearization
            %   computeLinearization()
            %   Computes the parametric matrices A, B, p, C, D, and q, with
            %   parameters (xbar,ubar), associated with the linearized system
            %
            %   x(k+1)/dot(x) = A(tbar,xbar,ubar) x(k) + B(tbar,xbar,ubar) u(k) + p(t,tbar,xbar,ubar)
            %   y(k)          = C(tbar,xbar,ubar) x(k) + D(tbar,xbar,ubar) u(k) + q(t,tbar,xbar,ubar)
            %
            %   By default these matrices are computed using the Symbolic
            %   Toolbox of Matlab and stored in the object as function handles.
            %
            %   computeLinearization('Sampled')
            %   In this case the matrices are computed using via sampling of
            %   the function. This mode is advised when the computation of the
            %   symbolic jacobians are prohibitive.
            %
            %   WARNING: At the moment this applies only to time invariant system
            %            The linearization is evaluated at t=0
            
            methodNotSpecified = (nargin == 1);
            
            if methodNotSpecified
                varargin{1} = 'Symbolic';
                methodNotSpecified = 0;
            end
            
            if not(methodNotSpecified) & ischar( varargin{1})
                
                switch varargin{1}
                    case 'Sampled'
                        
                        %% Compute linearizations
                        if isa(obj.f,'function_handle')
                            
                            fprintf(getMessage('GeneralSystem:LinearizingStateEquationS'));
                            
                            obj.A = @(tbar,xbar,ubar) jacobianSamples(@(x)obj.f(tbar,x,ubar),xbar);
                            obj.B = @(tbar,xbar,ubar) jacobianSamples(@(u)obj.f(tbar,xbar,u),ubar);
                            DtF   = @(tbar,xbar,ubar) jacobianSamples(@(t)obj.f(t,xbar,ubar),tbar);
                            
                            obj.p = @(t,tbar,xbar,ubar) ( DtF(tbar,xbar,ubar)*t + obj.f(tbar,xbar,ubar) - [DtF(tbar,xbar,ubar),obj.A(tbar,xbar,ubar),obj.B(tbar,xbar,ubar)]*[tbar;xbar;ubar] );
                            
                            fprintf(getMessage('done'));
                            
                        end
                        
                        if isa(obj.h, 'function_handle')
                            
                            fprintf(getMessage('GeneralSystem:LinearizingOutputEquationS'));
                            
                            if nargin(obj.h)==2
                                
                                DtH   = @(tbar,xbar) jacobianSamples(@(t)obj.h(t,xbar),tbar);
                                obj.C = @(tbar,xbar) jacobianSamples(@(x)obj.h(tbar,x),xbar);
                                obj.q = @(t,tbar,xbar) ( DtH(tbar,xbar)*t + obj.h(tbar,xbar) - [DtH(tbar,xbar),obj.C(tbar,xbar)]*[tbar;xbar] );
                                
                            elseif nargin(obj.h)==3
                                
                                DtH   = @(tbar,xbar,ubar) jacobianSamples(@(t)obj.h(t,xbar,ubar),tbar);
                                obj.C = @(tbar,xbar,ubar) jacobianSamples(@(x)obj.h(tbar,x,ubar),xbar);
                                obj.D = @(tbar,xbar,ubar) jacobianSamples(@(u)obj.h(tbar,xbar,u),ubar);
                                
                                obj.q = @(t,tbar,xbar,ubar) ( DtH(tbar,xbar,ubar)*t + obj.h(tbar,xbar,ubar) - [DtH(tbar,xbar,ubar),obj.C(tbar,xbar,ubar),obj.D(tbar,xbar,ubar)]*[tbar;xbar;ubar] );
                                
                            end
                            
                            fprintf(getMessage('done'));
                            
                        end
                        
                        
                    otherwise
                        
                        %% Compute linearizations
                        t = sym('t',[1,1]);
                        t = sym(t,'real');
                        
                        tbar = sym('tbar',[1,1]);
                        tbar = sym(tbar,'real');
                        
                        xbar = sym('xbar',[obj.nx,1]);
                        xbar = sym(xbar,'real');
                        
                        ubar = sym('ubar',[obj.nu,1]);
                        ubar = sym(ubar,'real');
                        
                        
                        
                        if isa(obj.f,'function_handle')
                            
                            fprintf(getMessage('GeneralSystem:LinearizingStateEquation'));
                            
                            DtF   = matlabFunction(  jacobian(obj.f(tbar,xbar,ubar),tbar), 'vars', {tbar,xbar,ubar});
                            
                            obj.A = matlabFunction(  jacobian(obj.f(tbar,xbar,ubar),xbar), 'vars', {tbar,xbar,ubar});
                            
                            obj.B = matlabFunction(  jacobian(obj.f(tbar,xbar,ubar),ubar), 'vars', {tbar,xbar,ubar});
                            
                            %obj.p = @(t,x,u,tbar,xbar,ubar) ( DtF(tbar,xbar,ubar)*t + obj.f(tbar,xbar,ubar) - [DtF(tbar,xbar,ubar);obj.A(tbar,xbar,ubar);obj.B(tbar,xbar,ubar)]*[tbar;xbar;ubar] );
                            
                            obj.p = matlabFunction( DtF(tbar,xbar,ubar)*t + obj.f(tbar,xbar,ubar) - [DtF(tbar,xbar,ubar),obj.A(tbar,xbar,ubar),obj.B(tbar,xbar,ubar)]*[tbar;xbar;ubar]   ,'vars',{t,tbar,xbar,ubar});
                            
                            fprintf(getMessage('done'));
                            
                        end
                        
                        if isa(obj.h, 'function_handle')
                            
                            fprintf(getMessage('GeneralSystem:LinearizingOutputEquation'));
                            
                            if nargin(obj.h)==2
                                
                                DtH   = matlabFunction(  jacobian(obj.h(tbar,xbar),tbar)  ,'vars',{tbar,xbar});
                                
                                obj.C = matlabFunction(  jacobian(obj.h(tbar,xbar),xbar)  ,'vars',{tbar,xbar});
                                
                                obj.q = matlabFunction( DtH(tbar,xbar)*t + obj.h(tbar,xbar) - [DtH(tbar,xbar),obj.C(tbar,xbar)]*[tbar;xbar]   ,'vars',{t,tbar,xbar});
                                
                            elseif nargin(obj.h)==3
                                
                                DtH   = matlabFunction(  jacobian(obj.h(tbar,xbar,ubar),tbar)  ,'vars',{tbar,xbar,ubar});
                                
                                obj.C = matlabFunction(  jacobian(obj.h(tbar,xbar,ubar),xbar)  ,'vars',{tbar,xbar,ubar});
                                
                                obj.D = matlabFunction(  jacobian(obj.h(tbar,xbar,ubar),ubar)  ,'vars',{tbar,xbar,ubar});
                                
                                obj.q = matlabFunction( DtH(tbar,xbar,ubar)*t + obj.h(tbar,xbar,ubar) - [DtH(tbar,xbar,ubar),obj.C(tbar,xbar,ubar),obj.D(tbar,xbar,ubar)]*[tbar;xbar;ubar]   ,'vars',{t,tbar,xbar,ubar});
                                
                            end
                            
                            
                            fprintf(getMessage('done'));
                            
                        end
                end
            end
        end
        
        
        
        function params = getParameters(obj)
            
            params = {...
                'nx', obj.nx, ...
                'nu', obj.nu, ...
                'ny', obj.ny,...
                'StateEquation', obj.f, ...
                'OutputEquation',obj.h, ...
                'InitialCondition',obj.initialConditions...
                ...'LinearizationMatrices',{obj.A,obj.B,obj.p,obj.C,obj.D,obj.q}...
                };
        end
        
        function preCondition(obj,tBar,xBar,uBar,diagonalMode,l)
            %% Precondition the optimization problem for l around the point
            % xHat and uHat   TODO:test
            
            
            %% Compute hessians
            %TODO: use eall the stage cost
            Qx = jacobianSamples(@(xBarInner)jacobianSamples(@(x)l(tBar,x,uBar),xBarInner),xBar);
            Qu = jacobianSamples(@(uBarInner)jacobianSamples(@(u)l(tBar,xBar,u),uBarInner),uBar);
            Qt = jacobianSamples(@(tBarInner)jacobianSamples(@(t)l(t,xBar,uBar),tBarInner),tBar);
            %x = getSymbolicRealVariable('x',3);
            %u = getSymbolicRealVariable('u',2);
            %jacobian(jacobian(obj.stageCost(x,uBar),x),x)
            
            %% Compute conditioners
            dimNullx = length(Qx)-rank(Qx);
            [Ux,Sx,Vx]=svd(Qx);
            
            diagSx = diag(Sx);
            diagSx(diagSx==0) = ones(size(diagSx(diagSx==0)));
            Sx = diag(diagSx);
            
            dimNullu = length(Qu)-rank(Qu);
            [Uu,Su,Vu]=svd(Qu);
            
            diagSu = diag(Su);
            diagSu(diagSu==0)= ones(size(diagSu(diagSu==0)));
            Su = diag(diagSu);
            
            
            dimNullt = length(Qt)-rank(Qt);
            [Ut,St,Vt]=svd(Qt);
            
            diagSt = diag(St);
            diagSt(diagSt==0) = ones(size(diagSt(diagSt==0)));
            St = diag(diagSt);
            
            if dimNullt == 0
                At = sqrt(St)*Vt';
            else
                At =[];
            end
            if dimNullx == 0
                Ax = sqrt(Sx)*Vx';
            else
                Ax =[];
            end
            if dimNullu == 0
                Au = sqrt(Su)*Vu';
            else
                Au =[];
            end
            
            %             if (nargin >3)&&diagonalMode
            %                 %Closest diagonal version (to ckeep the box sets, boxes)
            %                 Ax = diag(diag(chol(Ax'*Ax)));
            %                 Au = diag(diag(chol(Au'*Au)));
            %                 At = diag(diag(chol(At'*At)));
            %             end
            
            obj.changeOfCoordinate(Ax,[],Au,[],At,[]);
            
        end
        
        function  changeOfCoordinate(obj,varargin)
            %changeOfCoordinate Perform a change of state and/or input coordinate
            %
            % The new system are will be expressed in the new state/input coordinate frame
            % x' = A x+b
            % u' = C u+d
            % t' = e u+f
            %
            % Calling the function:
            %
            % sys.changeOfCoordinate(A,b,C,d,e,f)
            %
            % Example:
            % -----------------------------------------------------------------
            % sys = CtSystem(...
            %     'StateEquation' ,@(t,x,u) t + x+u,'nx',1,'nu',1,...
            %     'OutputEquation',@(t,x) t + x,'ny',1);
            %
            % xDot = sys.f(1,3,7) % 11
            % y    = sys.h(1,3)   % 4
            %
            % % x' = x+2, u' = 2*u+4, t' = 2*t-1
            %
            % sys.changeOfCoordinate(1,2,2,4,2,-1);
            %
            %
            % % \dot{x'} = t+x+u = (1/2)(t'+1) + x'- 2 + (1/2)u'-2
            % % y        = t+x-u = (1/2)(t'+1) + x'- 2
            %
            % xDot = sys.f(1,3,7) % (1/2)(1+1) + 3-2 + (1/2)7 -2 = 3.5
            % y    = sys.h(1,3)   % (1/2)(1+1) + 3-2  = 2
            % -----------------------------------------------------------------
            
            A = eye(obj.nx);
            b = zeros(obj.nx,1);
            C = eye(obj.nu);
            d = zeros(obj.nu,1);
            e = 1;
            f = 0;
            
            if nargin > 1 && not(isempty(varargin{1}))
                A = varargin{1};
            end
            if nargin > 2 && not(isempty(varargin{2}))
                b = varargin{2};
            end
            
            if nargin > 3 && not(isempty(varargin{3}))
                C = varargin{3};
            end
            
            if nargin > 4 && not(isempty(varargin{4}))
                d = varargin{4};
            end
            
            if nargin > 5 && not(isempty(varargin{5}))
                e = varargin{5};
            end
            
            if nargin > 6 && not(isempty(varargin{6}))
                f = varargin{6};
            end
            
            if not(isempty(obj.f))
                fOld = obj.f;
                obj.f = @(t,x,u) A*fOld(e\(t-f),A\(x-b),C\(u-d));
            end
            
            if not(isempty(obj.h))
                hOld = obj.h;
                obj.h = @(t,x) hOld(e\(t-f),A\(x-b));
            end
            
            obj.cA = A;
            obj.cb = b;
            obj.cC = C;
            obj.cd = d;
            obj.ce = e;
            obj.cf = f;
            
        end
        %% vertcat
        % function testVertcat
        %     clc;close all; clear all;
        %
        %     v1 = Unicycle();
        %     c1 = UniGoToPoint([1;1]);
        %
        %     v2 = Unicycle();
        %     c2 = UniGoToPoint(-[1;1]);
        %
        %     v12 = [v1;v2];
        %     v12.controller = InlineController(@(t,x) [c1.computeInput(t,x(1:3));
        %                                               c2.computeInput(t,x(4:6)) ] );
        %     v12.initialConditions = [0;0;0;0;0;0];
        %     dt = 0.1;
        %     va = VirtualArena(v12,...
        %           'StoppingCriteria'  ,@(i,as)i>50/dt,...
        %           'StepPlotFunction'  ,@ someStepPlotFunction, ...
        %           'DiscretizationStep',dt,...
        %           'PlottingFrequency' ,1/dt);
        %     va.run()
        % end
        %
        % function h = someStepPlotFunction(systemsList,log,oldHandles,k)
        %     x = log{1}.stateTrajectory(:,1:k-1);
        %     h(1) = plot(x(1,:),x(2,:)); hold on
        %     h(2) = plot(x(4,:),x(5,:));
        % end
        function ret = vertcat(a,b)
            
            if not(isa(a,'GeneralSystem') & isa(b,'GeneralSystem'))
                error(getMessage('GeneralSystem:vercat'));
            end
            
            if isa(a,'CtSystem')
                
                ret = CtSystem(...
                    'nx',a.nx+b.nx,...
                    'ny',a.ny+b.ny,...
                    'nu',a.nu+b.nu,...
                    'StateEquation', @(t,x,u)  [a.f(t,x(1:a.nx),u(1:a.nu));b.f(t,x(a.nx+1:a.nx+b.nx),u(a.nu+1:a.nu+b.nu))]...
                    );
            elseif isa(a,'DtSystem')
                ret = DtSystem(...
                    'nx',a.nx+b.nx,...
                    'ny',a.ny+b.ny,...
                    'nu',a.nu+b.nu,...
                    'StateEquation', @(t,x,u)  [a.f(t,x(1:a.nx),u(1:a.nu));b.f(t,x(a.nx+1:a.nx+b.nx),u(a.nu+1:a.nu+b.nu))]);
            else
                error(getMessage('GeneralSystem:vercat2'));
            end
            
            if not(isempty(a.h)) && not(isempty(b.h))
                ret.h =  @(t,x) [a.h(t,x(1:a.nx));b.h(t,x(a.nx+1:a.nx+b.nx))];
            elseif  not(isempty(a.h)) &&  isempty(b.h)
                ret.h =  @(t,x) a.h(t,x(1:a.nx));
            elseif not(isempty(b.h)) &&  isempty(a.h)
                ret.h =  @(t,x) b.h(t,x(a.nx+1:a.nx+b.nx));
            end
            
            
        end
        
        
    end
    
     methods(Static)
        function testComputeLinearization()
            sys = CtSystem(...
                'StateEquation', @(t,x,u) t+2*x+3*u,...
                'OutputEquation',@(t,x,u) 4*t+5*x+6*u,...
                'nx',1,'nu',1,'ny',1 ...
                );
            
            
            t    = 10*randn(1);
            zbar = 10*randn(3,1);
            
            sys.computeLinearization('Sampled');
            
            A1 = sys.A(zbar(1),zbar(2),zbar(3));
            B1 = sys.B(zbar(1),zbar(2),zbar(3));
            C1 = sys.C(zbar(1),zbar(2),zbar(3));
            D1 = sys.D(zbar(1),zbar(2),zbar(3));
            
            p1 = sys.p(t,zbar(1),zbar(2),zbar(3));
            q1 = sys.q(t,zbar(1),zbar(2),zbar(3));
            
            
            sys.computeLinearization();
            
            
            A2 = sys.A(zbar(1),zbar(2),zbar(3));
            B2 = sys.B(zbar(1),zbar(2),zbar(3));
            C2 = sys.C(zbar(1),zbar(2),zbar(3));
            D2 = sys.D(zbar(1),zbar(2),zbar(3));
            
            p2 = sys.p(t,zbar(1),zbar(2),zbar(3));
            q2 = sys.q(t,zbar(1),zbar(2),zbar(3));
            
            A1
            B1
            C1
            D1
            p1
            q1
            A1-A2
            B1-B2
            C1-C2
            D1-D2
            p1-p2
            q1-q2
            
            
            
            sys = CtSystem(...
                'StateEquation', @(t,x,u) t+2*x+3*u,...
                'OutputEquation',@(t,x) 4*t+5*x,...
                'nx',1,'nu',1,'ny',1 ...
                );
            
            
            t    = 10*randn(1);
            zbar = 10*randn(3,1);
            
            sys.computeLinearization('Sampled');
            
            A1 = sys.A(zbar(1),zbar(2),zbar(3));
            B1 = sys.B(zbar(1),zbar(2),zbar(3));
            C1 = sys.C(zbar(1),zbar(2));
            
            p1 = sys.p(t,zbar(1),zbar(2),zbar(3));
            q1 = sys.q(t,zbar(1),zbar(2));
            
            
            sys.computeLinearization();
            
            
            A2 = sys.A(zbar(1),zbar(2),zbar(3));
            B2 = sys.B(zbar(1),zbar(2),zbar(3));
            C2 = sys.C(zbar(1),zbar(2));
            
            p2 = sys.p(t,zbar(1),zbar(2),zbar(3));
            q2 = sys.q(t,zbar(1),zbar(2));
            
            A1
            B1
            C1
            p1
            q1
            A1-A2
            B1-B2
            C1-C2
            p1-p2
            q1-q2
            
        end
    end
end