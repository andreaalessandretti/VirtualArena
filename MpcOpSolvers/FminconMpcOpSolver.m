classdef FminconMpcOpSolver < MpcOpSolver
    %FminconMpcOpSolver implementation of a MpcOpSolver using matlab fmincon
    %   function.
    %
    %   See also MpcOpSolver
    properties
        
        sizeNonlinearTerminalConstratins
        
        sizeNonlinearStageConstratins
        
        nNonlinearTerminalConstratins
        
        nNonlinearStageConstratins
        
    end
    
    methods
        
        function obj = FminconMpcOpSolver(varargin)
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        
                        
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            
        end
        
        function ret = solve(obj,mpcOP,x0,warmStart,varargin)
            
            solverParameters = varargin;
            
            N  = mpcOP.horizonLength;
            
            nx = mpcOP.system.nx;
            
            nu = mpcOP.system.nu;
            
            switch class(mpcOP)
                
                case 'DtMpcOp'
                    
                    
                otherwise
                    
                    error('Only DtMpcOp supporteds')
                    
            end
            
            tic
            
            if not(isempty(warmStart))
                initVal = reshape(warmStart.u,N*nu,1);
            else
                initVal = 0.1*randn(mpcOP.system.nu*mpcOP.horizonLength,1);
            end
            
            
            posOpt = find(strcmp(solverParameters,'SolverOptions'));
            
            if(posOpt)
                options = solverParameters{posOpt+1};
            else
                options = optimset('Algorithm','sqp','Display','notify');
            end
            
            A = []; b = []; Aeq = []; beq = [];
            
            [Uopt,fval,exitflag,output,lambda,grad,hessian] = fmincon(...
                @(U) fminconCost(mpcOP,x0,U),...
                initVal,...
                A, b, Aeq, beq,...
                [],[],...
                @(U) obj.getNonlinearConstraints(mpcOP,x0,U),...
                options...
                );
            
            solution.xmin = Uopt;
            solution.fval = fval;
            solution.exitflag = exitflag;
            solution.output = output;
            solution.lambda = lambda;
            solution.grad = grad;
            solution.hessian = hessian;
            ret.matrixCompositionTime = 0;
            ret.solverTime = toc;
            ret.timeConditioning = 0;
            ret.timeOther = 0;
            
            
            switch lower(exitflag)
                case 1
                    %First-order optimality measure was less than options.TolFun,
                    %and maximum constraint violation was less than options.TolCon.
                    OK = 1;
                case 0
                    %Number of iterations exceeded options.
                    %MaxIter or number of function evaluations exceeded options.MaxFunEvals.
                    OK = 1;
                case -1
                    %The output function terminated the algorithm.
                    OK = 1;
                case -2
                    % No feasible point was found.
                    OK = 1;
                case 2
                    % (trust-region-reflective and interior-point algorithms):
                    %Change in x was less than options.TolX and maximum constraint violation was less than options.TolCon.
                    OK = 1;
                case 3
                    %trust-region-reflective
                    % Change in the objective function value was less than
                    % options.TolFun and maximum constraint violation was
                    % less than options.TolCon.
                    OK = 1;
                case 4
                    %Magnitude of the search direction was less than 2*options.TolX and maximum constraint violation was less than options.TolCon.
                    OK = 1;
                case 5
                    %Magnitude of directional derivative in search direction was less than 2*options.TolFun and maximum constraint violation was less than options.TolCon.
                    OK = 1;
                case -3
                    %(interior-point and sqp algorithms)
                    %Objective function at current iteration went below options.ObjectiveLimit and maximum constraint violation was less than options.TolCon.
                    OK = 1;
            end
            
            
            %ret.x_opt = reshape( xmin(1:nx*(N+1)    ,:)  ,nx,N+1);
            
            ret.u_opt = reshape(Uopt',nu,N);
            ret.x_opt = mpcOP.system.getStateTrajectory(x0,ret.u_opt);
            ret.solution = solution;
            ret.problem = not(OK);
            
            ret.solverParameters = {'SolverOptions',options};
            
            
            
            
        end
        
        function init(obj,mpcOP)
            
            stageConstratins = mpcOP.stageConstraints;
            
            terminalConstraints = mpcOP.terminalConstraints;
            
            %% Count constraints for memory allocation
            
            
            nNonlinearStageConstratins    = 0;
            
            sizeNonlinearStageConstratins = 0;
            
            for i =1:length(stageConstratins)
                
                if isa(stageConstratins{i},'GeneralSet')
                    
                    nNonlinearStageConstratins    = nNonlinearStageConstratins +1;
                    
                    sizeNonlinearStageConstratins = sizeNonlinearStageConstratins ...
                        + stageConstratins{i}.nf;
                else
                    error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                end
                
                
            end
            
            
            nNonlinearTerminalConstratins    = 0;
            sizeNonlinearTerminalConstratins = 0;
            
            for i =1:length(terminalConstraints)
                
                if isa(terminalConstraints{i},'GeneralSet')
                    nNonlinearTerminalConstratins    = nNonlinearTerminalConstratins +1;
                    
                    nNonlinearTerminalConstratins    = nNonlinearTerminalConstratins +1;
                    
                    sizeNonlinearTerminalConstratins = sizeNonlinearTerminalConstratins ...
                        + terminalConstraints{i}.nf;
                else
                    error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                end
                
            end
            
            
            obj.nNonlinearTerminalConstratins = nNonlinearTerminalConstratins;
            
            obj.nNonlinearStageConstratins = nNonlinearStageConstratins;
            
            obj.sizeNonlinearTerminalConstratins = sizeNonlinearTerminalConstratins;
            
            obj.sizeNonlinearStageConstratins = sizeNonlinearStageConstratins;
            
        end
        
        function faceProblem(obj,solution,mpcOp,xim1)
            error('problem');
        end
        
        function close(obj)
        end
        
        
        
        function [C,Ceq] = getNonlinearConstraints(obj,mpcOP,x0,U)
            
            nu = mpcOP.system.nu;
            N  = mpcOP.horizonLength;
            
            u = reshape(U',nu,N);
            x = mpcOP.system.getStateTrajectory(x0,u);
            
            terminalConstratins = mpcOP.terminalConstraints;
            stageConstratins    = mpcOP.stageConstraints;
            
            C = zeros(N*obj.sizeNonlinearStageConstratins+obj.sizeNonlinearTerminalConstratins,1);
            
            j = 1;
            
            for i =1:length(stageConstratins)
                
                if (isa(stageConstratins{i},'GeneralSet'))
                    for k = 1:N
                        ck = stageConstratins{i}.f([x(:,k);u(:,k)]);
                        nck = length(ck);
                        
                        C(j-1+(1:nck)) = ck;
                        j = j+nck;
                    end
                else
                    error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                end
            end
            
            %% Terminal Constraints
            
            for i =1:length(terminalConstratins)
                if (isa(terminalConstratins{i},'GeneralSet'))
                    ck = terminalConstratins{i}.f(x(:,k));
                    nck = length(ck);
                    
                    C(j-1+(1:nck)) = ck;
                    j = j+nck;
                else
                    error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                end
            end
            Ceq = [];
            
        end
        
        
    end
end

% U = [u1;u2;...]
function cost = fminconCost(dtMpcOp,x0,U)

cost = 0;
x  = x0;
nu = dtMpcOp.system.nu;

hasStageCost = not(isempty(dtMpcOp.stageCost));
hasTerminalCost = not(isempty(dtMpcOp.terminalCost));

for i =1:length(U)/nu
    
    u = U((i-1)*nu+(1:nu));
    
    if hasStageCost
        cost = cost + dtMpcOp.stageCost(x,u);
    end
    
    x = dtMpcOp.system.f(x,u);
    
end

if hasTerminalCost
    cost = cost + dtMpcOp.terminalCost(x);
end

if sum(sum(isnan(x)))>0 || sum(sum(isinf(x)))>0
    error('inf or nan prediction');
end

end
