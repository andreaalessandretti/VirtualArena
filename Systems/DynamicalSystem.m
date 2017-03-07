classdef DynamicalSystem < handle & InitDeinitObject & LinearizedSystem
    %DynamicalSystem
    %
    % x' = f(t,x,u)
    % y  = h(t,x)/h(t,x,u)
    %
    % x is an nx-dimensional vector
    % u is an nu-dimensional vector
    % y is an ny-dimensional vector
    %
    % (x',x,u,y) = (x(k+1),x(k),u(k),y(k))) in the discrte time case.
    % (x',x,u,y) = (\dot{x}(t),x(t),u(t),y(t)) in the continuous time case.
    %
    %  Use:
    % 
    %  sys = DynamicalSystem(par1,val1,par2,val2,...) % only by a subclass
    %
    %  where the parameters are chosen among the following
    %
    %   'nx', 'nu', 'ny', 'InitialCondition', 'Controller',
    %   'StateEquation', 'OutputEquation'  (See f and h, respectively, above)
    %   'LinearizationMatrices' (value: {A,B,p,C,D,q})
    %
    % DynamicalSystem properties:
    %
    % nx,nu,ny     - dimension of vectors x,u, and y respectively
    % f,h          - function handles of f(x,u)/f(x,u,w) and h(x,u)/h(x,u,w)
    %
    % A,B,p,C,D,q  - function handles @(tbar,xbar,ubar) of the parameters 
    %                of the linearized model
    %
    % x(k+1)/dot(x) = A(tbar,xbar,ubar) x(k) + B(tbar,xbar,ubar) u(k) + p(t,tbar,xbar,ubar)
    % y(k)          = C(tbar,xbar,ubar) x(k) + D(tbar,xbar,ubar) u(k) + q(t,tbar,xbar,ubar)
    %
    % initialCondition - initial condition/conditions of the system
    % stateObserver    - state observer (of class StateObserver)
    % controller       - system controller of class Controller, CtSystem, or
    %                     DtSystem
    % x                - current state vector
    %
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
        
        % it is posible to specify different initial conditions for multiple
        % simulations. In this case initialCondition(:,i) containts the
        % ith initial condition to simulate
        initialCondition
        
        stateObserver
        
        controller;      %controller used to drive the vehicle
        x;               %current state vector
        y;               %current output vector
        
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
        
        info
        
    end
    
   methods (Abstract) 
        f(varargin)  % State equation ( function handle @(t,x,u) )
        
        %h(varargin)  % Output equation ( function handle @(t,x,u) )
   end
    
   methods
        function y =  h(obj,t,x,u)
            y = x;
        end
        
        function obj = DynamicalSystem (varargin)
            %
            %       sys = DynamicalSystem(par1,val1,par2,val2,...)
            %  where the parameters are chosen among the following
            %
            %   'nx', 'nu', 'ny', 'InitialCondition', 'Q', 'R', 'Controller',
            %   'StateEquation', 'OutputEquation'  (See f and h, respectively, above)
            %   'LinearizationMatrices' (value: {A,B,p,C,D,q})
            
            %% Retrive parameters for superclass DynamicalSystem
            
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
                            
                        case 'InitialCondition'
                            
                            obj.initialCondition = varargin{parameterPointer+1};
                            
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
          
        function params = getParameters(obj)
            
            params = {...
                'nx', obj.nx, ...
                'nu', obj.nu, ...
                'ny', obj.ny,...
                'InitialCondition',obj.initialCondition...
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
        %     v12.controller = IController(@(t,x) [c1.computeInput(t,x(1:3));
        %                                               c2.computeInput(t,x(4:6)) ] );
        %     v12.initialCondition = [0;0;0;0;0;0];
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
            
            if not(isa(a,'DynamicalSystem') & isa(b,'DynamicalSystem'))
                error(getMessage('DynamicalSystem:vercat'));
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
                error(getMessage('DynamicalSystem:vercat2'));
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
end