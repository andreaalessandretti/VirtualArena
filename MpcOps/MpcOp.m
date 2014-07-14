%MpcOp Model Predictive Control Optimization Problem
%
% Given a vector z in R^n$ and the horizon length T>0, the open loop MPC 
% optimization problem consists in finding the optimal control trajectory 
% u^*([0,T]), defined in the interval [0,T], that solves
% 
% J_T^*(z) = min_u([0,T])   J_T(z,u([0,T])) 
% s.t. dot{x}=f(x,u) ( or x+ =f(x,u) )
%      x(0) = z
%      x(T) \in X_a
%      x(tau) \in X and u(tau)\in U for tau in [0,T]
%  
% with
%
% J_T(x,u([0,T])) = int_0^T l(x(tau),u(tau)) dtau + m(x(T))
% 
% ( or J_T(x,u([1,T])) = sum_i=1^T l(x(i),u(i))  + m(x(T)) )
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
%                         e.g. GeneralSet, PolytopicSet, BoxSet
% 'TerminalConstraints' : some set on the [x;u] space (child of GeneralSet)
%                       : e.g. GeneralSet, PolytopicSet, BoxSet
% 'StageCost'           : function handle @(x,u) stage cost l(x,u)
% 'TerminalCost'        : function handle @(x) terminal cost m(x)
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
        
        % xbar = conditionerX*x, solve the problem in xbar
        conditionerX
        
        % ubar = conditionerU*u, solve the problem in xbar
        conditionerU
        
        Dl
        
        Hl
        
        Dm
        
        Hm
        
        stateTrajectoryLinearization;
        
        inputTrajectoryLinearization;
            
        hessianComputationMethod = 'Symbolic';
        
        jacobianComputationMethod = 'Symbolic';
        
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
            if ( isempty(obj.system) || isempty(obj.horizonLength) || ( isempty(obj.stageCost) && isempty(obj.terminalCost) ) )
                error('MpcOp:RequiredParametersMissing',getMessage('MpcOp:RequiredParametersMissing'))
            end
           
        end
        
        function params = getParameters(obj)
            params = {...
                'System',obj.system, ...
                'HorizonLength', obj.horizonLength,...
                'StageConstraints', obj.stageConstraints,...
                'TerminalConstraints', obj.terminalConstraints,...
                'StageCost', obj.stageCost,...
                'TerminalCost', obj.terminalCost,...
                'InputDerivative', obj.inputDerivative,...
                'AuxiliaryLaw', obj.auxiliaryLaw...
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
            
            fprintf(getMessage('MpcOp:evaluation'));
            

            obj.stageCost    = matlabFunction(simplify( obj.stageCost(x,u) ) ,'vars',{x,u});
            obj.terminalCost = matlabFunction(simplify( obj.terminalCost(x) ) ,'vars',{x});
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
                    
                    z  = [x;u];
                    l  = obj.stageCost(x,u);
                    
                    obj.Dl = matlabFunction(  jacobian(l,z)  ,'vars',{x,u} );
           
            
                case 'AA'
                    
                    obj.Dl = @(x,u) jacobianEstimationAA(@(z)obj.stageCost(z(1:nx),z(nx+1:end)),[x;u]);
                  
                otherwise 
                    error('Unknown method for the approximation of the hessian');
                    
                    
            end
            
             
            switch obj.hessianComputationMethod
                case 'Symbolic'
                    
                    x = sym('x',[nx,1]);
                    x = sym(x,'real');
                    
                    u = sym('u',[nu,1]);
                    u = sym(u,'real');
                    
                    z = [x;u];
                    l  = obj.stageCost(x,u);
                    
                    obj.Hl = matlabFunction(  jacobian(jacobian(l,z),z)  ,'vars',{x,u} );
                    
                case 'AA'
                    
                    obj.Hl = @(x,u) jacobianEstimationAA(@(z)obj.Dl(z(1:nx),z(nx+1:end)),[x;u]);
                
                case 'Identity'
                    
                    obj.Hl = @(x,u) eye(nx+nu);
                
                otherwise 
                    error('Unknown method for the approximation of the hessian');
            end
            
            if not(isempty(obj.terminalCost))
             switch obj.jacobianComputationMethod
                case 'Symbolic'
                    
                    x = sym('x',[nx,1]);
                    x = sym(x,'real');
                    
                    m  = obj.terminalCost(x);
                    obj.Dm = matlabFunction(jacobian(m,x),'vars',{x});
            
                case 'AA'
                    
                    obj.Dm = @(x) jacobianEstimationAA(obj.terminalCost,x);
            
                otherwise 
                    error('Unknown method for the approximation of the hessian');
             end
            
            
             switch obj.hessianComputationMethod
                case 'Symbolic'
                    
                    x = sym('x',[nx,1]);
                    x = sym(x,'real');
                    
                    m  = obj.terminalCost(x);
                    obj.Hm = matlabFunction(jacobian(jacobian(m,x),x),'vars',{x});
            
                case 'AA'
                    %obj.Hm = @(x) hessianEstimationAA(obj.Dm,x);
                    obj.Hm = @(x) jacobianEstimationAA(obj.Dm,x);
                case 'Identity'
                    
                    obj.Hm = @(x) eye(nx);
                
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
        
        
        % Sompute the matrixes A and B such that
        % A*[x1,x2,...,xN+1,u1,u2,...,uN]'=b 
        % denote the dynamic equation constraints of the linearized system
        % %% TO COMMENT
        function [A,b] = computeEqualityDynamicConstraints(obj)
            
             N      = obj.horizonLength;
             nx     = obj.system.nx;
             nu     = obj.system.nu;
            
            lin_x  = obj.stateTrajectoryLinearization(:,1:N+1);
            lin_u  = obj.inputTrajectoryLinearization(:,1:N);
            
                cs   = zeros(nx*N,1);
                bkAs = zeros(nx*N,nx*N);
                bkBs = zeros(nx*N,nu*N);
                for i = 1:N
                    
                    xb = lin_x(:,i);
                    ub = lin_u(:,i);
                    
                    Aj = obj.system.A(xb,ub);
                    Bj = obj.system.B(xb,ub);
                    gj = obj.system.p(xb,ub);
                    
                    bkAs((i-1)*nx+[1:nx],(i-1)*nx+[1:nx]) = Aj;
                    
                    bkBs((i-1)*nx+[1:nx],(i-1)*nu+[1:nu]) = Bj;
                    
                    cs((i-1)*nx+[1:nx],1) = gj;
                end
            
                 
            A = - [bkAs,zeros(N*nx, nx+nu*N)] ...
                + [zeros(nx*N,nx),eye(nx*N),zeros(nx*N,nu*N)]...
                - [zeros(nx*N,nx*(N+1)),bkBs];
            
            b = cs;
            
        end
        
        
        % Precondition the optimization problem around the point
        % xHat and uHat %% TO COMMENT
        function preCondition(obj,xBar,uBar,diagonalMode)
            
            %% Compute hessians
            %TODO: use eall the stage cost
            Qx = jacobianEstimationAA(@(xBarInner)jacobianEstimationAA(@(x)obj.stageCost(x,uBar),xBarInner),xBar);
            Qu = jacobianEstimationAA(@(uBarInner)jacobianEstimationAA(@(u)obj.stageCost(xBar,u),uBarInner),uBar);
            
            %x = getSymbolicRealVariable('x',3);
            %u = getSymbolicRealVariable('u',2);
            %jacobian(jacobian(obj.stageCost(x,uBar),x),x)
            
            %% Compute conditioners
            dimNullx = length(Qx)-rank(Qx);
            [Ux,Sx,Vx]=svd(Qx);
            
            diagSx = diag(Sx);
            diagSx(diagSx==0)= ones(size(diagSx(diagSx==0)));
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
            
            
            obj.conditionerX = Ax;
            obj.conditionerU = Au;
            
            %% Apply conditioners
            oldStageCost = obj.stageCost;
            obj.stageCost = @(x,u)oldStageCost(Ax\x,Au\u);
            
            oldTerminalCost = obj.terminalCost;
            obj.terminalCost = @(x)oldTerminalCost(Ax\x);
            
            oldStageConstraints = obj.stageConstraints;
            
            for i=1:length(oldStageConstraints)
                sc = oldStageConstraints{i};
                obj.stageConstraints{i}= blkdiag(Ax,Au)*sc;
            end
            
            oldTerminalConstraints = obj.terminalConstraints;
            
            for i=1:length(oldTerminalConstraints)
                sc = oldTerminalConstraints{i};
                obj.terminalConstraints{i}= Ax*sc;
            end
            
            obj.system.stateInputTransformation(Ax,Au);
            
            
            if not(isempty(obj.auxiliaryLaw))
                oldAl = obj.auxiliaryLaw;
                
                obj.auxiliaryLaw =@(x) Au*oldAl(Ax\x);
            end
            
        end
    end
end