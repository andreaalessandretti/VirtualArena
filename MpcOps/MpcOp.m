%MpcOp Model Predictive Control Optimization Problem
%
% Given a vector z in R^n$ and the horizon length T>0, the open loop MPC
% optimization problem consists in finding the optimal control trajectory
% u^*([0,T]), defined in the interval [0,T], that solves
%
% J_T^*(t0,x0) = min_u([t0,t0+T])   J_T(x0,t0,u([t0,t0+T]))
% s.t. dot{x}=f(t,x,u) ( or x+ =f(t,x,u) )
%      x(t0) = x0
%      x(t0+T) \in X_a(t)
%      x(tau) \in X(t) and u(tau)\in U(t) for tau in [t0,t0+T]
%
% with
%
% J_T(t0,x0,u([t0,t0+T])) = int_0^T l(tau,x(tau),u(tau)) dtau + m(t0+T,x(t0+T))
%
% ( or J_T(t0,x0,u([t0,t0+T])) = sum_i=t0^t0+T l(i,x(i),u(i))  + m(t0+T,x(t0+T)) )
%
% The finite horizon cost J_T(.) is composed of the stage cost l(.) and
% the terminal cost m(.), which is defined over the auxiliary terminal set
% X_a.
%
% CtMpcOp(par1,val1,...) ( or DtMpcOp(par1,val1,...) )
%
% Parameters:
%
% 'System'              : CtSytem (or DtSytem)
% 'HorizonLength'       : positive real (or integer)
% 'StageConstraints'    : some set on the [x;u] space (child of GeneralSet)
%                         or [t;x;u] space
%                         e.g. GeneralSet, PolytopicSet, BoxSet
% 'TerminalConstraints' : some set on the [x] space (child of GeneralSet)
%                         or [t;x] space
%                       : e.g. GeneralSet, PolytopicSet, BoxSet
% 'StageCost'           : function handle @(t,x,u) stage cost l(t,x,u)
% 'TerminalCost'        : function handle @(t,x) terminal cost m(t,x)
%
%
% e.g.
% mpcOp = CtMpcOp( ...
%     'System'               , v1,...
%     'HorizonLength'        , 0.5,...
%     'StageConstraints'     , BoxSet( -[1;pi/4],4:5,[1;pi/4],4:5,5),... % on the variable z=[x;u];
%     'StageCost'            , @(t,x,u) x(1:2)'*x(1:2) + u'*u,...
%     'TerminalCost'         , @(t,x) 10*x(1:2)'*x(1:2),...
%     'TerminalConstraints'  , BoxSet( -100*ones(3,1),1:3,100*ones(3,1),1:3,3)... % on the variable x(T);
%     );
%
% see also GeneralSet, PolytopicSet, BoxSet, CtMpcOp, DtMpcOp

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

classdef MpcOp < handle
    
    properties
        
        system
        
        horizonLength
        
        stageConstraints = {}
        
        terminalConstraints = {}
        
        stageCost
        
        terminalCost
        
        inputDerivative
        
        auxiliaryLaw = [];
        
        Dl
        
        Hl
        
        Dm
        
        Hm
        
        stateTrajectoryLinearization;
        
        inputTrajectoryLinearization;
        
        hessianComputationMethod = 'Symbolic';
        
        jacobianComputationMethod = 'Symbolic';
        
        cA, cb, cC, cd, ce, cf %Change of coordinate
    end
    
    methods
        function obj = MpcOp(varargin)
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'InputDerivative'
                            
                            obj.inputDerivative = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'System'
                            
                            obj.system = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'HorizonLength'
                            
                            obj.horizonLength = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StageConstraints'
                            
                            par = varargin{parameterPointer+1};
                            
                            if iscell(par) % The constraints are stored in cells
                                
                                obj.stageConstraints = par;
                                
                            else
                                
                                obj.stageConstraints = {par};
                                
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'TerminalConstraints'
                            
                            
                            par = varargin{parameterPointer+1};
                            
                            if iscell(par)
                                
                                obj.terminalConstraints = par;
                                
                            else
                                
                                obj.terminalConstraints = {par};
                                
                            end
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StageCost'
                            
                            obj.stageCost = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'TerminalCost'
                            
                            obj.terminalCost = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'AuxiliaryLaw'
                            
                            obj.auxiliaryLaw = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            %% Check required parameters
            %            if ( isempty(obj.system) || isempty(obj.horizonLength) || ( isempty(obj.stageCost) && isempty(obj.terminalCost) ) )
            %                error('MpcOp:RequiredParametersMissing',getMessage('MpcOp:RequiredParametersMissing'))
            %            end
            
        end
        
        function params = getParameters(obj)
            params = {...
                'System'             , obj.system, ...
                'HorizonLength'      , obj.horizonLength,...
                'StageConstraints'   , obj.stageConstraints,...
                'TerminalConstraints', obj.terminalConstraints,...
                'StageCost'          , obj.stageCost,...
                'TerminalCost'       , obj.terminalCost,...
                'InputDerivative'    , obj.inputDerivative,...
                'AuxiliaryLaw'       , obj.auxiliaryLaw...
                };
        end
        
        function useSymbolicEvaluation(obj)
            %%useSymbolicEvaluation evaluates the state and output equations
            %   with symbolic variables, simplify them, and replace them with the
            %   simplified version.
            %
            %   This can lead to a notable decrease of computation time for
            %   the case of complex models.
            
            obj.system.useSymbolicEvaluation();
            
            x = sym('x',[obj.system.nx,1]);
            x = sym(x,'real');
            
            u = sym('u',[obj.system.nu,1]);
            u = sym(u,'real');
            
            t = sym('t',[1,1]);
            t = sym(t,'real');
            
            fprintf(getMessage('MpcOp:evaluation'));
            
            
            obj.stageCost    = matlabFunction(simplify( obj.stageCost(t,x,u) ) ,'vars',{t,x,u});
            obj.terminalCost = matlabFunction(simplify( obj.terminalCost(t,x) ) ,'vars',{t,x});
            fprintf(getMessage('done'));
        end
        
        %% TO COMMENT
        function Hm = HmBfgs(obj,xk,xkm1,Dmkm1,Hmkm1)
            
            dx = xk-xkm1;
            
            y = (obj.Dm(xk)-Dmkm1)';
            s = dx;
            
            if norm(s)>0
                V = - Hmkm1*(s*s')*Hmkm1/(s'*Hmkm1*s);
            else
                V = zeros(length(s),length(s));
            end
            
            if norm(y)>0
                U = y*y'/(y'*s);
            else
                U = zeros(length(y),length(y));
            end
            
            Hm = Hmkm1 + U + V;
            
        end
        
        %% TO COMMENT
        function Hl = HlBfgs(obj,xuk,xukm1,Dlkm1,Hlkm1)
            
            nx = obj.system.nx;
            nu = obj.system.nu;
            
            dx = xuk-xukm1;
            
            y = (obj.Dl(xuk(1:nx),xuk(nx+1:nx+nu))-Dlkm1)';
            s = dx;
            
            if norm(s)>0
                V =  - Hlkm1*(s*s')*Hlkm1/(s'*Hlkm1*s);
            else
                V = zeros(length(s),length(s));
            end
            
            
            if norm(y)>0
                U = y*y'/(y'*s);
            else
                U = zeros(length(y),length(y));
            end
            
            Hl = Hlkm1 +U +V;
            
        end
        
        %% TO COMMENT
        function computeLinearization(obj)
            
            % Check folder for code generation
            
            nx = obj.system.nx;
            
            nu = obj.system.nu;
            
            if isempty(obj.system.A)
                obj.system.computeLinearization();
            end
            
            %% Symbolic variables/functions for Jacobian/Hessian computation
            
            
            
            %% Stage cost Approximation
            fprintf(getMessage('MpcOpSolver:LinearizinStageCost'));
            
            
            switch obj.jacobianComputationMethod
                case 'Symbolic'
                    
                    x = sym('x',[nx,1]);
                    x = sym(x,'real');
                    
                    u = sym('u',[nu,1]);
                    u = sym(u,'real');
                    
                    t = sym('t',[1,1]);
                    t = sym(t,'real');
                    
                    z  = [t;x;u];
                    l  = obj.stageCost(t,x,u);
                    
                    obj.Dl = matlabFunction(  jacobian(l,z)  ,'vars',{t,x,u} );
                    
                    
                case 'AA'
                    
                    obj.Dl = @(t,x,u) jacobianEstimationAA(@(z)obj.stageCost(z(1),z(2:nx+1),z(nx+2:end)),[t;x;u]);
                    
                otherwise
                    error('Unknown method for the approximation of the hessian');
                    
                    
            end
            
            
            switch obj.hessianComputationMethod
                case 'Symbolic'
                    
                    x = sym('x',[nx,1]);
                    x = sym(x,'real');
                    
                    u = sym('u',[nu,1]);
                    u = sym(u,'real');
                    
                    t = sym('t',[1,1]);
                    t = sym(t,'real');
                    
                    z = [t;x;u];
                    l  = obj.stageCost(t,x,u);
                    
                    obj.Hl = matlabFunction(  jacobian(jacobian(l,z),z)  ,'vars',{t,x,u} );
                    
                case 'AA'
                    
                    obj.Hl = @(t,x,u) jacobianEstimationAA(@(z)obj.Dl(z(1),z(2:nx+1),z(nx+2:end)),[t;x;u]);
                    
                case 'Identity'
                    
                    obj.Hl = @(t,x,u) eye(nx+nu+1);
                    
                otherwise
                    error('Unknown method for the approximation of the hessian');
            end
            
            if not(isempty(obj.terminalCost))
                switch obj.jacobianComputationMethod
                    case 'Symbolic'
                        
                        x = sym('x',[nx,1]);
                        x = sym(x,'real');
                        
                        t = sym('t',[1,1]);
                        t = sym(t,'real');
                        
                        m  = obj.terminalCost(t,x);
                        obj.Dm = matlabFunction(jacobian(m,x),'vars',{t,x});
                        
                    case 'AA'
                        
                        obj.Dm = @(t,x) jacobianEstimationAA(@(z)obj.terminalCost(z(1),z(2:end)),[t;x]);
                        
                    otherwise
                        error('Unknown method for the approximation of the hessian');
                end
                
                
                switch obj.hessianComputationMethod
                    case 'Symbolic'
                        
                        x = sym('x',[nx,1]);
                        x = sym(x,'real');
                        
                        t = sym('t',[1,1]);
                        t = sym(t,'real');
                        
                        m  = obj.terminalCost(t,x);
                        z = [t;x];
                        obj.Hm = matlabFunction(jacobian(jacobian(m,z),z),'vars',{t,x});
                        
                    case 'AA'
                        
                        obj.Hm = @(t,x) jacobianEstimationAA(@(z)obj.Dm(z(1),z(2:end)),[t;x]);
                        
                    case 'Identity'
                        
                        obj.Hm = @(t,x) eye(nx+1);
                        
                    otherwise
                        error('Unknown method for the approximation of the hessian');
                end
                
            end
            fprintf(getMessage('done'));
            
            
            %% Linearization of the terminal constratints
            fprintf(getMessage('MpcOpSolver:LinearizingTerminalConstraint'));
            
            for i = 1:length(obj.terminalConstraints)
                
                con = obj.terminalConstraints{i};
                
                if( strcmp(class(con),'GeneralSet') )
                    
                    obj.terminalConstraints{i} = LinearizedSet(obj.terminalConstraints{i});
                    
                end
                
            end
            
            fprintf(getMessage('done'));
            
            if (isempty(obj.stateTrajectoryLinearization))
                
                obj.stateTrajectoryLinearization = 0.01*randn(obj.system.nx,obj.horizonLength+1);
                
            end
            
            if (isempty(obj.inputTrajectoryLinearization))
                
                obj.inputTrajectoryLinearization = 0.01*randn(obj.system.nu,obj.horizonLength);
                
            end
            
        end
        
        
        function preCondition(obj,xBar,uBar,diagonalMode)
        %% Precondition the optimization problem around the point
        % xHat and uHat   TODO:test
            %% Compute hessians
            %TODO: use eall the stage cost
            Qx = jacobianEstimationAA(@(xBarInner)jacobianEstimationAA(@(x)obj.stageCost(0,x,uBar),xBarInner),xBar);
            Qu = jacobianEstimationAA(@(uBarInner)jacobianEstimationAA(@(u)obj.stageCost(0,xBar,u),uBarInner),uBar);
            
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
            
            Ax = sqrt(Sx)*Vx';
            Au = sqrt(Su)*Vu';
            
            if (nargin ==4)&&diagonalMode
                %Closest diagonal version (to ckeep the box sets, boxes)
                Ax = diag(diag(chol(Ax'*Ax)));
                Au = diag(diag(chol(Au'*Au)));
            end
            
            obj.changeOfCoordinate(Ax,[],Au,[])
            
        end
        
        
        function changeOfCoordinate(obj,varargin)
            
            %changeOfCoordinate Perform a change of state and/or input coordinate
            %
            % The new system are will be expressed in the new state/input coordinate frame
            % x' = A x+b
            % u' = C u+d
            % t' = e u+f
            %
            
            A = eye(obj.system.nx);
            b = zeros(obj.system.nx,1);
            C = eye(obj.system.nu);
            d = zeros(obj.system.nu,1);
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
            
            
            %% Apply conditioners
            oldStageCost = obj.stageCost;
            obj.stageCost = @(t,x,u)oldStageCost(e\(t-f),A\(x-b),Au\u);
            
            oldTerminalCost = obj.terminalCost;
            obj.terminalCost = @(t,x)oldTerminalCost(e\(t-f),A\(x-b));
            
            oldStageConstraints = obj.stageConstraints;
            
            for i=1:length(oldStageConstraints)
                sc = oldStageConstraints{i};
                obj.stageConstraints{i}= blkdiag(A,C)*sc;
            end
            
            oldTerminalConstraints = obj.terminalConstraints;
            
            for i=1:length(oldTerminalConstraints)
                sc = oldTerminalConstraints{i};
                obj.terminalConstraints{i}= A*sc;
            end
            
            obj.system.changeOfCoordinate(A,b,C,d,e,f);
            
            if not(isempty(obj.auxiliaryLaw))
                oldAl = obj.auxiliaryLaw;
                
                obj.auxiliaryLaw =@(x) C*oldAl(A\(x-b));
            end
            
            obj.cA = A;
            obj.cb = b;
            obj.cC = C;
            obj.cd = d;
            obj.ce = e;
            obj.cf = f;
            
        end
        
        
        
    end
end