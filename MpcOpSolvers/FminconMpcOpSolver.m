classdef FminconMpcOpSolver < MpcOpSolver & InitDeinitObject
    %FminconMpcOpSolver implementation of a MpcOpSolver using matlab fmincon
    %   function.
    %
    %   In order to use specific setting use the 'SolverOptions' as in the
    %   following example:
    %
    %   ...
    %   opSolver = FminconMpcOpSolver('MpcOp',op);
    % 
    %   sys.controller = MpcController(...
    %       'MpcOp'                 , op,...
    %       'MpcOpSolver'           , opSolver,...
    %       'MpcOpSolverParameters' , {'SolverOptions', optimset('Algorithm','sqp','Display','notify','MaxIter',4)} ...
    %     );
    %   ...
    %
    %   See also MpcOpSolver
    properties
        
        sizeNonlinearTerminalConstratins
        
        sizeNonlinearStageConstratins
        
        nNonlinearTerminalConstratins
        
        nNonlinearStageConstratins
        
        mpcOp
        
    end
    
    methods
        
        function obj = FminconMpcOpSolver(varargin)
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'MpcOp'
                            
                            obj.mpcOp = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            if isempty(obj.mpcOp)
                error(getMessage('FminconMpcOpSolver:NoMpcOp'));
            end
            
        end
        
        function ret = solve(obj,mpcOp,k0,x0,warmStart,varargin)
            
            mpcOp = obj.mpcOp;
            
            solverParameters = varargin;
            
            N  = mpcOp.horizonLength;
            
            nx = mpcOp.system.nx;
            
            nu = mpcOp.system.nu;
            
            switch class(mpcOp)
                
                case 'DtMpcOp'
                    
                otherwise
                    
                    error('Only DtMpcOp supporteds')
                    
            end
            
            tic
            
            if not(isempty(warmStart))
                initVal = reshape(warmStart.u,N*nu,1);
            else
                initVal = 0.1*randn(mpcOp.system.nu*mpcOp.horizonLength,1);
            end
            
            posOpt = find(strcmp(solverParameters,'SolverOptions'));
            
            if(posOpt)
                options = solverParameters{posOpt+1};
            else
                options = optimset('Algorithm','sqp','Display','notify');
            end
            
            A = []; b = []; Aeq = []; beq = [];
            
            [Uopt,fval,exitflag,output,lambda,grad,hessian] = fmincon(...
                @(U) fminconCost(mpcOp,k0,x0,U),...
                initVal,...
                A, b, Aeq, beq,...
                [],[],...
                @(U) obj.getNonlinearConstraints(mpcOp,k0,x0,U),...
                options...
                );
            
            solution.xmin             = Uopt;
            solution.fval             = fval;
            solution.exitflag         = exitflag;
            solution.output           = output;
            solution.lambda           = lambda;
            solution.grad             = grad;
            solution.hessian          = hessian;
            ret.matrixCompositionTime = 0;
            ret.solverTime            = toc;
            ret.timeConditioning      = 0;
            ret.timeOther             = 0;
            
            exitflag
            switch lower(exitflag)
                case 1
                    %First-order optimality measure was less than options.TolFun,
                    %and maximum constraint violation was less than options.TolCon.
                    OK = 1;
                case 0
                    %Number of iterations exceeded options.
                    %MaxIter or number of function evaluations exceeded options.MaxFunEvals.
                    OK = 0;
                case -1
                    %The output function terminated the algorithm.
                    OK = 0;
                case -2
                    % No feasible point was found.
                    OK = 0;
                case 2
                    % (trust-region-reflective and interior-point algorithms):
                    %Change in x was less than options.TolX and maximum constraint violation was less than options.TolCon.
                    OK = 1;
                case 3
                    %trust-region-reflective
                    % Change in the objective function value was less than
                    % options.TolFun and maximum constraint violation was
                    % less than options.TolCon.
                    OK = 0;%1;
                case 4
                    %Magnitude of the search direction was less than 2*options.TolX and maximum constraint violation was less than options.TolCon.
                    OK = 0;%1;
                case 5
                    %Magnitude of directional derivative in search direction was less than 2*options.TolFun and maximum constraint violation was less than options.TolCon.
                    OK = 0;%1
                case -3
                    %(interior-point and sqp algorithms)
                    %Objective function at current iteration went below options.ObjectiveLimit and maximum constraint violation was less than options.TolCon.
                    OK = 0;%1;
            end
            
            
            %ret.x_opt = reshape( xmin(1:nx*(N+1)    ,:)  ,nx,N+1);
            
            ret.u_opt     = reshape(Uopt',nu,N);
            ret.x_opt     = mpcOp.system.getStateTrajectory(k0,x0,ret.u_opt);
            
            ret.tu_opt    = k0:k0+N-1;
            ret.tx_opt    = k0:k0+N;
            
            ret.solution  = solution;
            ret.problem   = not(OK);
            
            ret.solverParameters = {'SolverOptions',options};
            
            
            
            
        end
        
        function initSimulations(obj)
            
            mpcOp = obj.mpcOp;
            stageConstratins    = mpcOp.stageConstraints;
            
            terminalConstraints = mpcOp.terminalConstraints;
            
            %% Count constraints for memory allocation
            
            
            nNonlinearStageConstratins    = 0;
            
            sizeNonlinearStageConstratins = 0;
            
            for i =1:length(stageConstratins)
                
                if isa(stageConstratins{i},'GeneralSet')
                    
                    nNonlinearStageConstratins    = nNonlinearStageConstratins + 1;
                    
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
        
        
        function sol = faceProblem(obj,mpcController,problematicSol,t,x,varargin)
            error('problem');
        end
        
        function close(obj)
        end
        
        function [C,Ceq] = getNonlinearConstraints(obj,mpcOp,k0,x0,U)
            
            nu = mpcOp.system.nu;
            nx = mpcOp.system.nx;
            N  = mpcOp.horizonLength;
            
            u = reshape(U',nu,N);
            x = mpcOp.system.getStateTrajectory(k0,x0,u);
            
            terminalConstratins = mpcOp.terminalConstraints;
            stageConstratins    = mpcOp.stageConstraints;
            
            C = zeros(N*obj.sizeNonlinearStageConstratins+obj.sizeNonlinearTerminalConstratins,1);
            
            j = 1;
            
            for i =1:length(stageConstratins)
                con = stageConstratins{i};
                if (isa(con,'GeneralSet'))
                    k = k0;
                    for kk = 1:N
                        
                        if con.nx == nx+ nu
                            ckk = con.f([x(:,kk);u(:,kk)]);
                        elseif con.nx == nx+ nu +1
                            ckk = con.f([k;x(:,kk);u(:,kk)]);
                        end
                        nckk = length(ckk);
                        
                        C(j-1+(1:nckk)) = ckk;
                        j = j+nckk;
                        k = k + 1;
                    end
                else
                    error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                end
            end
            
            %% Terminal Constraints
            
            for i =1:length(terminalConstratins)
                con = terminalConstratins{i};
                if (isa(con,'GeneralSet'))
                    
                       if con.nx == nx
                            ckk = con.f(x(:,N+1));
                        elseif con.nx == nx +1
                            ckk = con.f([k;x(:,N+1)]);
                        end
                        
                    nckk = length(ckk);
                    
                    C(j-1+(1:nckk)) = ckk;
                    j = j+nckk;
                else
                    error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                end
            end
            Ceq = [];
            
        end
        
        
    end
end

% U = [u1;u2;...]
function cost = fminconCost(dtMpcOp,k0,x0,U)

cost = 0;
x = x0;
k = k0;

nu = dtMpcOp.system.nu;

hasStageCost    = not(isempty(dtMpcOp.stageCost));
hasTerminalCost = not(isempty(dtMpcOp.terminalCost));

for i =1:length(U)/nu
    
    u = U((i-1)*nu+(1:nu));
    
    if hasStageCost
        cost = cost + dtMpcOp.stageCost(k,x,u);
    end
    
    x = dtMpcOp.system.f(k,x,u);
    k = k + 1;
end

if hasTerminalCost
    cost = cost + dtMpcOp.terminalCost(k,x);
end

if sum(sum(isnan(x)))>0 || sum(sum(isinf(x)))>0
    error('inf or nan prediction');
end

end
