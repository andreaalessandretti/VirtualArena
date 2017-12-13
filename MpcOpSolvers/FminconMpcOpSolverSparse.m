classdef FminconMpcOpSolverSparse < FminconMpcOpSolver & InitDeinitObject
%     function example
% clc; close all; clear all;
% 
% dt = 0.2;
% 
% %% Unicycle Model
% sys = ICtSystem(...
%     'StateEquation', @(t,x,u,varargin) [
%     u(1)*cos(x(3));
%     u(1)*sin(x(3));
%     u(2)],...
%     'nx',3,'nu',2 ...
% );
% 
% sys.initialCondition = {[15;15;-pi/2],-[15;15;-pi/2],[15;-15;pi],[-15;15;-pi/2]};
% 
% auxiliaryControlLaw =  TrackingController_ECC13(... % <<< attached to the realSystem
%     @(t) 10*[sin(0.1*t); cos(0.1*t)] , ... % c
%     @(t)    [cos(0.1*t);-sin(0.1*t)] , ... % cDot
%     eye(2)                           , ... % K
%     [1;0] );
% 
% e = @(t,x)auxiliaryControlLaw.computeError(t,x);
% 
% mpcOp = ICtMpcOp( ...
%     'System'               , sys,...
%     'HorizonLength'        , 3*dt,...
%     'StageConstraints'     , BoxSet( -[1;1],4:5,[1;1],4:5,5),... % on the variable z=[x;u];
%     'StageCost'            , @(t,x,u,varargin) e(t,x)'* e(t,x),...
%     'TerminalCost'         , @(t,x,varargin) 0.3333*(e(t,x)'* e(t,x))^(3/2)...
%     );
% 
% dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);
% 
% dtRealSystem = DiscretizedSystem(sys,dt);
% 
% dtRealSystem.controller = MpcController(...
%     'MpcOp'       , dtMpcOp ,...
%     'MpcOpSolver' , FminconMpcOpSolverSparse('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
%     );
% 
% va = VirtualArena(dtRealSystem,...
%     'StoppingCriteria'  , @(t,sysList)t>70/dt,...
%     'PlottingStep'      , 1/dt, ...
%     'StepPlotFunction'  , @ex01StepPlotFunction ...
%     );
% 
% log = va.run();
% end 
% function h = ex01StepPlotFunction(sysList,log,plot_handles,k)
% 
% logX = log{1}.stateTrajectory(:,1:k); hold on;
% h    = plot(logX(1,:),logX(2,:));
% grid on
% 
% end
% 

    methods
        
        function obj = FminconMpcOpSolverSparse(varargin)
            obj = obj@FminconMpcOpSolver(varargin{:});
        end
        
        %% Overwrite FminconMpcOpSolver
        % U = [x1;...;xN;u0;...;uN-1]
        function cost = fminconCost(obj,dtMpcOp,k0,x0,U,netReadings)
            
            N  = dtMpcOp.horizonLength;
            nx = dtMpcOp.system.nx;
            nu = dtMpcOp.system.nu;
            
            xTraj = [x0,reshape(U(1:N*nx),nx,N)];
            uTraj = reshape(U(N*nx+(1:N*nu)),nu,N);
            kTraj = k0+(0:N);
            
            cost  = 0;
            
            for i=1:N
                cost = cost + dtMpcOp.stageCost(kTraj(i),xTraj(:,i),uTraj(i));
            end
            
            cost = cost + dtMpcOp.terminalCost(kTraj(N+1),xTraj(:,N+1));
            
        end
        %% Overwrite FminconMpcOpSolver
        function Ceq = getNonlinearConstraintsCeq(obj,mpcOp,k0,x0,U,netReadings)
            
            N  = obj.mpcOp.horizonLength;
            nx = obj.mpcOp.system.nx;
            nu = obj.mpcOp.system.nu;
            
            xTraj = [x0,reshape(U(1:N*nx),nx,N)];
            uTraj = reshape(U(N*nx+(1:N*nu)),nu,N);
            kTraj = k0+(0:N);
            
            Ceq = [];
            
            for i =1:N
                Ceq = [Ceq; xTraj(:,i+1)-obj.mpcOp.system.f(kTraj(i),xTraj(:,i),uTraj(:,i))];
            end
            
        end
        %% Overwrite FminconMpcOpSolver
        function C = getNonlinearConstraintsC(obj,mpcOp,k0,x0,U,netReadings)
            
            N  = obj.mpcOp.horizonLength;
            nx = obj.mpcOp.system.nx;
            nu = obj.mpcOp.system.nu;
            
            xTraj = [x0,reshape(U(1:N*nx),nx,N)];
            uTraj = reshape(U(N*nx+(1:N*nu)),nu,N);
            kTraj = k0+(0:N);
            
            C   = [];
            
            for ii = 1:N
                
                k    = kTraj(ii);
                x_k  = xTraj(:,ii);
                u_k  = uTraj(:,ii);
                
                for i_2 =1:length(obj.mpcOp.stageConstraints)
                    con = obj.mpcOp.stageConstraints{i_2};
                    if (isa(con,'GeneralSet'))
                        if con.nx == nx + nu
                            
                            if nargin(con.f)==1
                                ckk = con.f([x_k;u_k]);
                            else
                                error('Network Constraints not supported.')
                            end
                            
                        elseif con.nx == nx + nu + 1
                            
                            if nargin(con.f) == 1
                                ckk = con.f([k;x_k;u_k]);
                            else
                                error('Network Constraints not supported.')
                            end
                            
                        else
                            error('The dimension of the constraints set is not valid')
                        end
                        
                        C = [C;ckk];
                    else
                        error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                    end
                end
            end
            
            k_N  = kTraj(N+1);
            x_N  = xTraj(:,N+1);
            
            %% Terminal Constraints
            for i =1:length(obj.mpcOp.terminalConstraints)
                con = obj.mpcOp.terminalConstraints{i};
                if (isa(con,'GeneralSet'))
                    
                    if con.nx == nx
                        ckk = con.f(x_N);
                    elseif con.nx == nx +1
                        ckk = con.f([k_N;x_N]);
                    end
                    
                    C = [C;ckk];
                else
                    error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                end
            end
            
        end
        
        %% Overwrite FminconMpcOpSolver
        function [kTraj,xTraj,uTraj] = getStateTrajectoriesMultiMpc(obj,mpcOps,k0,x0,U,netReadings)
              
            N  = obj.mpcOp.horizonLength;
            nx = obj.mpcOp.system.nx;
            nu = obj.mpcOp.system.nu;
            
            xTraj = [x0,reshape(U(1:N*nx),nx,N)];
            uTraj = reshape(U(N*nx+(1:N*nu)),nu,N);
            kTraj = k0+(0:N);
            
        end
        
        %% Overwrite FminconMpcOpSolver
        function initVal = getInitialCondition(obj,warmStart,mpcOp)
            N  = mpcOp.horizonLength;
            nu = mpcOp.system.nu;
            nx = mpcOp.system.nx;
                if not(isempty(warmStart))
                    initVal = [reshape(warmStart.x_opt(:,2:end),N*nx,1);reshape(warmStart.u_opt,N*nu,1)];
                else
                    initVal = 0.1*randn(N*(nu+nx),1);
                end
        end
        
        %% Overwrite FminconMpcOpSolver
        function sizeU = getSizeOptimizer(obj)
              N  = obj.mpcOp.horizonLength;
              nu = obj.mpcOp.system.nu;
              nx = obj.mpcOp.system.nx;
             
              sizeU = N*(nu+nx);
        end
        
        
    end
    
end

