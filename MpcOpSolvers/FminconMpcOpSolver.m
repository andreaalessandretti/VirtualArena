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
    %
    %  Parameters:
    %  'UseSymbolicEvaluation',{0}. Cell of sizes of measuraments from the
    %  network. Thay can not change during the simulation.
    %
    %   See also MpcOpSolver
    
    properties
        
        
        mpcOp
        
        useSymbolicEvaluation = 0;
        dimNet
        fminconCostSym
        fminconConSym
        symCons
        
        solverTime =0;
    end
    
    methods(Static)
        function OK = isOK(exitflag)
            
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
        end
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
                            
                        case 'UseSymbolicEvaluation'
                            
                            
                            if varargin{parameterPointer+1} == 1
                                obj.useSymbolicEvaluation = 1;
                                obj.dimNet = {1};
                            elseif varargin{parameterPointer+1} == 0
                                obj.useSymbolicEvaluation = 0;
                            else
                                obj.useSymbolicEvaluation = 1;
                                obj.dimNet = varargin{parameterPointer+1};
                            end
                            
                            
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
            
            solverParameters = varargin;
            
            if length(varargin)>=1
                netReadings = varargin{1};
            else
                netReadings = [];
            end
            
            %% Solver Options
            posOpt = find(strcmp(solverParameters,'SolverOptions'));
            
            if(posOpt)
                options = solverParameters{posOpt+1};
            else
                options = optimset('Algorithm','sqp','Display','notify');
            end
            
            if iscell(mpcOp)
                
                %% Multi-mpp folmulation mpcOps = {mpcOp1,delta1,...}
                mpcOps = mpcOp;
                Nmpcs = length(mpcOps);
                
                nOptVars = 0;
                
                %% Check that are all DtSystems and sum nu and N
                for i=1:Nmpcs
                    mpcOpi = mpcOps{i};
                    %deltai = mpcOps{2+2*(i-1)};
                    if not(isa(mpcOpi,'DtMpcOp'))
                        error('Only DtMpcOp supporteds')
                    end
                    nOptVars = nOptVars + mpcOpi.system.nu*mpcOpi.horizonLength;
                end
                
                %% Build Initial Condition
                if not(isempty(warmStart))
                    initVal = reshape(warmStart.u,size(warmStart.u,1)*size(warmStart.u,2),1);
                else
                    initVal = [];
                    for i=1:Nmpcs
                        mpcOpi  = mpcOps{i};
                        initVal = [initVal;
                            0.1*randn( mpcOpi.system.nu*mpcOpi.horizonLength,1)];
                    end
                end
            else
                
                %mpcOp = obj.mpcOp;
                
                switch class(mpcOp)
                    
                    case 'DtMpcOp'
                        
                    otherwise
                        
                        error('Only DtMpcOp supporteds')
                        
                end
                
                tic
                
                N  = mpcOp.horizonLength;
                
                nu = mpcOp.system.nu;
                
                %% Warm start
                if not(isempty(warmStart))
                    initVal = reshape(warmStart.u,N*nu,1);
                else
                    initVal = 0.1*randn(mpcOp.system.nu*mpcOp.horizonLength,1);
                end
                
            end
            
            %% Solve
            A = []; b = []; Aeq = []; beq = [];
            
            if obj.useSymbolicEvaluation
                cost = @(U) obj.fminconCostSym(k0,x0,U,netReadings);
                cons = @(U) obj.fminconConSym2(k0,x0,U,netReadings);
                
            else
                cost = @(U) obj.fminconCost(mpcOp,k0,x0,U,netReadings);
                cons = @(U) obj.getNonlinearConstraints(mpcOp,k0,x0,U,netReadings);
            end
            %cons = @(U) obj.getNonlinearConstraints(mpcOp,k0,x0,U,netReadings);
            tic
            [Uopt,fval,exitflag,output,lambda,grad,hessian] = fmincon(...
                cost,...
                initVal,...
                A, b, Aeq, beq,...
                [],[],...
                cons,...
                options...
                );
            
            %% Check Exit Flag
            fprintf('exitflag: %i\n',  exitflag);
            
            OK = FminconMpcOpSolver.isOK(exitflag);
            
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
            
            [kkk,xxx,uuu] = obj.getStateTrajectoriesMultiMpc(mpcOp,k0,x0,Uopt,netReadings);
            
            ret.u_opt     = uuu;
            ret.x_opt     = xxx;
            
            ret.tu_opt    = kkk;
            ret.tx_opt    = kkk;
            
            ret.solution  = solution;
            ret.problem   = not(OK);
            
            ret.solverParameters = {'SolverOptions',options};
            obj.solverTime = ret.solverTime;
        end
        
        function [C,Ceq] = fminconConSym2(obj,k0,x0,U,netReadings)
            C = obj.fminconConSym(k0,x0,U,netReadings);
            Ceq = [];
        end
        
        function initSimulations(obj)
            
            mpcOp = obj.mpcOp;
            obj.symbolizeProblem(mpcOp);
        end
        
        function symbolizeProblem(obj,dtMpcOp)
            
            
            if obj.useSymbolicEvaluation
                
                sprintf('Computing symbolic evaluation of the cost...');
                
                netDims = obj.dimNet;
                sizeU   = obj.getSizeOptimizer();
                sizeX0  = obj.getSizeInitialCondition();
                
                inputSizes = {1,sizeX0,sizeU,netDims};
                
                obj.fminconCostSym = symbolize(...
                    @(k0,x0,U,netReadings)obj.fminconCost(dtMpcOp,k0,x0,U,netReadings),...
                    inputSizes...
                    );
                
                
                obj.fminconConSym = symbolize(...
                    @(k0,x0,U,netReadings) obj.getNonlinearConstraints(dtMpcOp,k0,x0,U,netReadings),...
                    inputSizes...
                    );
                
                sprintf('done.\n');
                
            end
            
            
        end
        
        
        function sol = faceProblem(obj,mpcController,problematicSol,t,x,varargin)
            error('problem');
        end
        
        function close(obj)
        end
        
        function [C,Ceq] = getNonlinearConstraints(obj,mpcOp,k0,x0,U,netReadings)
            
            
            [kk,xx,uu] = obj.getStateTrajectoriesMultiMpc(mpcOp,k0,x0,U,netReadings);
            C = obj.evaluateConstraintsTrajectories(mpcOp,kk,xx,uu,netReadings);
            Ceq = [];
            
        end
        
        function C = evaluateConstraintsTrajectories(obj,mpcOps,kk,xx,uu,netReadings)
            
            if isa(mpcOps,'DtMpcOp')
                mpcOps = {mpcOps};
            end
            
            C   = [];
            Nmpcs  = length(mpcOps);
            
            nx = size(xx,1);
            nu = size(uu,1);
            
            %Size of the constaints associated with the measurament of the
            %network
            netConSize = 0;
            if not(isempty(netReadings))
                nSensors = length(netReadings);
                for ii = 1:nSensors
                    nNeigh = length(netReadings{ii});
                    sizeConOneSens = 2; % TODO: fix this hack
                    netConSize = netConSize + nNeigh*sizeConOneSens;
                end
            end
            
            
            for ii = 1:length(kk)
                
                k     = kk(ii);
                
                for jj = 1:Nmpcs
                    
                    %Extract Local Info
                    mpcOp_jj = mpcOps{jj};
                    H_jj     = mpcOps{jj}.horizonLength;
                    
                    
                    x_k  = xx(:,ii);
                    
                    %Check if the mpc_jj optimizes over the current
                    %time or if it uses the auxiliary law
                    if(ii<=H_jj)
                        
                        u_k  = uu(:,ii);
                        stageConstratins_jj    = mpcOp_jj.stageConstraints;
                        
                        for i_2 =1:length(stageConstratins_jj)
                            con = stageConstratins_jj{i_2};
                            if (isa(con,'GeneralSet'))
                                
                                if con.nx == nx + nu
                                    
                                    if nargin(con.f)==1
                                        ckk = con.f([x_k;u_k]);
                                    else
                                        ckk = con.f([x_k;u_k],netReadings);
                                    end
                                    
                                elseif con.nx == nx + nu + 1
                                    
                                    if nargin(con.f) == 1
                                        ckk = con.f([k;x_k;u_k]);
                                    else
                                        ckk = con.f([k;x_k;u_k], netReadings);
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
                    
                    if(ii==H_jj+1)
                        
                        terminalConstratins_jj    = mpcOp_jj.terminalConstraints;
                        
                        %% Terminal Constraints
                        for i =1:length(terminalConstratins_jj)
                            con = terminalConstratins_jj{i};
                            if (isa(con,'GeneralSet'))
                                
                                if con.nx == nx
                                    ckk = con.f(x_k);
                                elseif con.nx == nx +1
                                    ckk = con.f([k;x_k]);
                                end
                                
                                C = [C;ckk];
                            else
                                error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                            end
                        end
                        
                        performanceConstraints_jj    = mpcOp_jj.performanceConstraints;
                        
                        %% Performance Constraints
                        for i =1:length(performanceConstraints_jj)
                            con = performanceConstraints_jj{i};
                            if (isa(con,'GeneralSet'))
                                
                                if nargin(con.f) == 3 %con(J,t0,x0)
                                    Ji = obj.evaluateTrajectoriesJi(mpcOp_jj,kk,xx,uu,netReadings);
                                    
                                    ckk = con.f(kk(1),xx(:,1),Ji);
                                else
                                    error('Wrong size performanceConstraints_jj');
                                end
                                
                                C = [C;ckk];
                            else
                                error(getMessage('FminconMpcOpSolver:OnlyGeneralSet'));
                            end
                        end
                        
                    end
                end
            end
        end
        
        % U = [u1;u2;...]
        function cost = fminconCost(obj,dtMpcOp,k0,x0,U,netReadings)
            
            x = x0;
            
            if not(iscell(dtMpcOp))
                dtMpcOp = {dtMpcOp};
            end
            
            [kkk,xxx,uuu] = obj.getStateTrajectoriesMultiMpc(dtMpcOp,k0,x0,U,netReadings);
            
            if sum(sum(isnan(xxx)))>0 || sum(sum(isinf(xxx)))>0|| sum(sum(isnan(uuu)))>0 || sum(sum(isinf(uuu)))>0
                error('inf or nan prediction');
            end
            
            cost = obj.evaluateTrajectories(dtMpcOp,kkk,xxx,uuu,netReadings);
            
        end
        
        function [kk,xx,uu] = getStateTrajectoriesMultiMpc(obj,mpcOps,k0,x0,U,netReadings)
            
            if isa(mpcOps,'DtMpcOp')
                mpcOps = {mpcOps};
            end
            
            x = x0;
            
            Nmpcs  = length(mpcOps);
            
            cumulativePositionU = 0;
            
            ii = 0;
            
            atLeastOneMpcOpWithinHorizon = 1;
            
            xx(:,ii+1) = x;
            k          = k0 + ii;
            kk(ii+1)   = k0;
            while(atLeastOneMpcOpWithinHorizon)
                
                
                
                xNext = x; %Temporary - to fix dimentions
                
                cumulativePositionX = 0;
                
                atLeastOneMpcOpWithinHorizon = 0; %Temporary
                
                uCollective = [];
                
                
                for jj = 1:Nmpcs
                    
                    %Extract Local Info
                    H_jj     = mpcOps{jj}.horizonLength;
                    
                    %Check if next time is within horizon
                    if(ii+2 <= H_jj)
                        atLeastOneMpcOpWithinHorizon = 1;
                    end
                end
                
                for jj = 1:Nmpcs
                    
                    %Extract Local Info
                    mpcOp_jj = mpcOps{jj};
                    H_jj     = mpcOps{jj}.horizonLength;
                    
                    
                    if not(isempty(mpcOp_jj.system)) ...
                            && not(isempty(mpcOp_jj.system.f)) ...
                            && not(isempty(mpcOp_jj.system.nx)) ...
                            && not(isempty(mpcOp_jj.system.nu))
                        nx_jj    = mpcOp_jj.system.nx;
                        nu_jj    = mpcOp_jj.system.nu;
                        f_jj     = mpcOp_jj.system.f;
                        selX_jj  = cumulativePositionX+(1:nx_jj);
                        
                        
                        %Check if the mpc_jj optimizes over the current
                        %time or if it uses the auxiliary law
                        if(ii<=H_jj-1)
                            
                            selU_jj = cumulativePositionU+(1:nu_jj);
                            
                            u_j_i   = U(selU_jj);
                            
                            cumulativePositionU = cumulativePositionU + nu_jj;
                            
                        elseif atLeastOneMpcOpWithinHorizon
                            
                            auxLaw_jj    = mpcOp_jj.auxiliaryLaw;
                            
                            if isempty(auxLaw_jj)
                                error('mpcOp_%i must have an auxiliary law ',jj);
                            end
                            
                            u_j_i = auxLaw_jj.computeInput(k,x(selX_jj));
                            
                        end
                        
                        if nargin(f_jj)==3
                            xNext(selX_jj) = f_jj(k,x(selX_jj),u_j_i);
                        elseif nargin(f_jj)==6
                            xNext(selX_jj) = f_jj(k,x(selX_jj),u_j_i,netReadings,ii+1,xx(:,1));
                        else
                            error('To many input for f()');
                        end
                        
                        uCollective = [uCollective;u_j_i];
                        
                        cumulativePositionX = cumulativePositionX + nx_jj;
                    end
                end
                
                uu(:,ii+1) = uCollective;
                
                ii = ii + 1;
                
                x  = xNext;
                
                k          = k0 + ii;
                xx(:,ii+1) = x;
                kk(ii+1)   = k;
            end
            
            
        end
        
        function n = getSizeInitialCondition(obj)
            
            mpcOps = obj.mpcOp;
            
            if isa(mpcOps,'DtMpcOp') && not(isempty(mpcOps.system))
                n = mpcOps.system.nx;
            elseif iscell(mpcOps)
                
                Nmpcs  = length(mpcOps);
                
                n = 0;
                
                for jj = 1:Nmpcs
                    if not(isempty(mpcOps{jj}.system)) && not(isempty(mpcOps{jj}.system.nx))
                        n = n + mpcOps{jj}.system.nx;
                    end
                end
                
            else
                error('Only cell of DtMpcOps or DtMpcOp are accepted.');
            end
        end
        function cumulativePositionU = getSizeOptimizer(obj)
            
            mpcOps = obj.mpcOp;
            
            cumulativePositionU = 0;
            
            if isa(mpcOps,'DtMpcOp')
                if not(isempty(mpcOps.system)) && not(isempty(mpcOps.system.nu))
                    cumulativePositionU = mpcOps.system.nu*mpcOps.horizonLength;
                end
                
            elseif iscell(mpcOps)
                
                
                Nmpcs  = length(mpcOps);
                
                cumulativePositionU = 0;
                
                ii = 0;
                
                atLeastOneMpcOpWithinHorizon = 1;
                
                while(atLeastOneMpcOpWithinHorizon)
                    
                    atLeastOneMpcOpWithinHorizon = 0; %Temporary
                    
                    for jj = 1:Nmpcs
                        
                        %Extract Local Info
                        mpcOp_jj = mpcOps{jj};
                        H_jj     = mpcOps{jj}.horizonLength;
                        if not(isempty(mpcOp_jj.system)) && not(isempty(mpcOp_jj.system.nu))
                            nu_jj    = mpcOp_jj.system.nu;
                            
                            
                            %Check if next time is within horizon
                            if(ii+1 <= H_jj)
                                atLeastOneMpcOpWithinHorizon = 1;
                            end
                            
                            %Check if the mpc_jj optimizes over the current
                            %time or if it uses the auxiliary law
                            if(ii<=H_jj-1) && not(isempty(nu_jj))
                                
                                cumulativePositionU = cumulativePositionU + nu_jj;
                                
                            end
                        end
                        
                    end
                    
                    ii = ii + 1;
                    
                end
                
            else
                error('Only cell of DtMpcOps or DtMpcOp are accepted.');
            end
            
        end
        
        function cost = evaluateTrajectories(obj,mpcOps,kk,xx,uu,netReadings)
            
            cost = 0;
            
            Nmpcs  = length(mpcOps);
            
            for ii = 1:length(kk)
                
                k     = kk(ii);
                
                for jj = 1:Nmpcs
                    
                        
                    %Extract Local Info
                    mpcOp_jj = mpcOps{jj};
                    H_jj     = mpcOps{jj}.horizonLength;
                    
                    if not(mpcOp_jj.neglectPerformanceIndex)
                    x_j_i  = xx(:,ii);
                    
                    %Check if the mpc_jj optimizes over the current
                    %time or if it uses the auxiliary law
                    if(ii<=H_jj)
                        u_j_i  = uu(:,ii);
                        stageCost_jj = mpcOp_jj.stageCost;
                        
                        if not(isempty(stageCost_jj))
                            if nargin(stageCost_jj) ==3
                                cost = cost + stageCost_jj(k,x_j_i,u_j_i);
                            elseif nargin(stageCost_jj) ==4
                                cost = cost + stageCost_jj(k,x_j_i,u_j_i,netReadings);
                            elseif nargin(stageCost_jj) ==5
                                cost = cost + stageCost_jj(k,x_j_i,u_j_i,netReadings,ii);
                            else %nargin(stageCost_jj) ==6
                                cost = cost + stageCost_jj(k,x_j_i,u_j_i,netReadings,ii,xx(:,1));
                                
                            end
                        end
                        
                    end
                    
                    if(ii==H_jj+1)
                        
                        terminalCost_jj = mpcOp_jj.terminalCost;
                        
                        if not(isempty(terminalCost_jj))
                            if nargin(terminalCost_jj) ==2
                                cost = cost + terminalCost_jj(k,x_j_i);
                            elseif nargin(terminalCost_jj) ==3
                                cost = cost + terminalCost_jj(k,x_j_i,netReadings);
                            else
                                cost = cost + terminalCost_jj(k,x_j_i,netReadings,xx(:,1));
                            end
                        end
                        
                    end
                    
                    end
                end
                
            end
        end
        
        function cost = evaluateTrajectoriesJi(obj,mpcOp_jj,kk,xx,uu,netReadings)
            
            cost = 0;
            
            for ii = 1:length(kk)
                
                k     = kk(ii);
                
                
                
                %Extract Local Info
                H_jj     = mpcOp_jj.horizonLength;
                
                
                x_j_i  = xx(:,ii);
                
                %Check if the mpc_jj optimizes over the current
                %time or if it uses the auxiliary law
                if(ii<=H_jj)
                    
                    u_j_i  = uu(:,ii);
                    
                    stageCost_jj = mpcOp_jj.stageCost;
                    
                    if not(isempty(stageCost_jj))
                        if nargin(stageCost_jj) ==3
                            cost = cost + stageCost_jj(k,x_j_i,u_j_i);
                        elseif nargin(stageCost_jj) ==4
                            cost = cost + stageCost_jj(k,x_j_i,u_j_i,netReadings);
                        elseif nargin(stageCost_jj) ==5
                            cost = cost + stageCost_jj(k,x_j_i,u_j_i,netReadings,ii);
                        else %nargin(stageCost_jj) ==6
                            cost = cost + stageCost_jj(k,x_j_i,u_j_i,netReadings,ii,xx(:,1));
                            
                        end
                    end
                    
                end
                
                if(ii==H_jj+1)
                    
                    terminalCost_jj = mpcOp_jj.terminalCost;
                    
                    if not(isempty(terminalCost_jj))
                        if nargin(terminalCost_jj) ==2
                            cost = cost + terminalCost_jj(k,x_j_i);
                        elseif nargin(terminalCost_jj) ==3
                            cost = cost + terminalCost_jj(k,x_j_i,netReadings);
                        else
                            cost = cost + terminalCost_jj(k,x_j_i,netReadings,xx(:,1));
                            
                        end
                    end
                    
                end
                
                
            end
        end
        
        function cost = evaluateTrajectoriesJiAllL(obj,mpcOp_jj,kk,xx,uu,netReadings)
            
            cost = [];
            
            for ii = 1:length(kk)
                
                k     = kk(ii);
                
                
                
                %Extract Local Info
                H_jj     = mpcOp_jj.horizonLength;
                
                
                x_j_i  = xx(:,ii);
                
                %Check if the mpc_jj optimizes over the current
                %time or if it uses the auxiliary law
                if(ii<=H_jj)
                    
                    u_j_i  = uu(:,ii);
                    
                    stageCost_jj = mpcOp_jj.stageCost;
                    
                    if not(isempty(stageCost_jj))
                        if nargin(stageCost_jj) ==3
                            cost = [cost, stageCost_jj(k,x_j_i,u_j_i)];
                        elseif nargin(stageCost_jj) ==4
                            cost = [cost,stageCost_jj(k,x_j_i,u_j_i,netReadings)];
                        elseif nargin(stageCost_jj) ==5
                            cost = [cost,stageCost_jj(k,x_j_i,u_j_i,netReadings,ii)];
                        else %nargin(stageCost_jj) ==6
                            cost = [cost,stageCost_jj(k,x_j_i,u_j_i,netReadings,ii,xx(:,1))];
                            
                        end
                    end
                    
                end
                
                if(ii==H_jj+1)
                    
                    terminalCost_jj = mpcOp_jj.terminalCost;
                    
                    if not(isempty(terminalCost_jj))
                        if nargin(terminalCost_jj) ==2
                            cost = [cost, terminalCost_jj(k,x_j_i)];
                        elseif nargin(terminalCost_jj) ==3
                            cost = [cost,terminalCost_jj(k,x_j_i,netReadings)];
                        else
                            cost = [cost, terminalCost_jj(k,x_j_i,netReadings,xx(:,1))];
                            
                        end
                    end
                    
                end
                
                
            end
        end
        
    end
end

