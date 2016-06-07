classdef AcadoMpcOpSolver < MpcOpSolver & InitDeinitObject
    %AcadoMpcOpSolver
    % e.g.
    %
    % mpcOpSolver = AcadoMpcOpSolver('StepSize',dt,'MpcOp',op,'AcadoOptimizationAlgorithmOptions', {...
    %       'KKT_TOLERANCE',1e-4,'MAX_NUM_ITERATIONS',30 ...
    %            });
    %
    %
    % Parameters :
    %
    % AcadoProblemName   (Default 'acadoCode')
    % AcadoNIntervals    (Default 10)
    % DisplayAcadoCode   (Default 1)
    % StepSize           Defines the acado OCP intervals as HorizonLength/StepSize
    % MpcOp              MpcOp to solve
    %
    % AcadoOptimizationAlgorithmOptions {'MAX_NUM_ITERATIONS',10,...}
    % AcadoMinimizeLSQ    if set to 1, l(x,u) = |stageCost(x,u)|^2 and the
    %                     terminal
    %
    % For more see http://acado.sourceforge.net/doc/html/index.html
    % MAX_NUM_ITERATIONS	    ini     maximum number of SQP iterations (maxNumIterations = 0: only simulation)
    % KKT_TOLERANCE	            double  termination tolerance for the optimal control algorithm
    % INTEGRATOR_TOLERANCE      double	the relative tolerance of the integrator
    % ABSOLUTE_TOLERANCE	    double	the absolute tolerance of the integrator ('ATOL')
    % MAX_NUM_INTEGRATOR_STEPS  int	    maximum number of integrator steps
    % LEVENBERG_MARQUARDT	    double	value for Levenberg-Marquardt regularization
    % MIN_LINESEARCH_PARAMETER  double	minimum stepsize of the line-search globalization
    %
    % HESSIAN_APPROXIMATION
    %     CONSTANT_HESSIAN  constant hessian (generalized gradient method)
    %     FULL_BFGS_UPDATE  BFGS update of the whole hessian
    %     BLOCK_BFGS_UPDATE structure exploiting BFGS update (default)
    %     GAUSS_NEWTON      Gauss-Newton Hessian approximation (only for LSQ)
    %     EXACT_HESSIAN     Exact Hessians
    %
    % DISCRETIZATION_TYPE
    %     SINGLE_SHOOTING   single shooting discretization
    %     MULTIPLE_SHOOTING multiple shooting discretization (default)
    %     COLLOCATION       collocation (will be implemented soon)
    %
    %
    % INTEGRATOR_TYPE
    %     INT_RK12 Runge Kutta integrator (adaptive Euler method)
    %     INT_RK23 Runge Kutta integrator (order 2/3, RKF )
    %     INT_RK45 Runge Kutta integrator (order 4/5, Dormand Prince)
    %     INT_RK78 Runge Kutta integrator (order 7/8, Dormand Prince)
    %     INT_BDF  BDF (backward differentiation formula) integrator
    %
    
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
        
        acadoProblemName = 'acadoCode';
        
        displayAcadoCode = 1;
        
        mpcOp
        
        acadoOptimizationAlgorithmOptions = {};
        
        acadoMinimizeLSQ = 0;
        
        stepSize;
        
        conditionProblem = @(out)not(out.CONVERGENCE_ACHIEVED)
        
        
    end
    
    methods
        
        function obj = AcadoMpcOpSolver(varargin)
            
            parameterPointer = 1;
            
            hasParameters    = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'DisplayAcadoCode'
                            
                            obj.displayAcadoCode = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'AcadoMinimizeLSQ'
                            
                            obj.acadoMinimizeLSQ = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StepSize'
                            
                            obj.stepSize = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'AcadoProblemName'
                            
                            obj.acadoProblemName = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'AcadoOptimizationAlgorithmOptions'
                            
                            obj.acadoOptimizationAlgorithmOptions = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
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
        end
        
        function ret = solve(obj,mpcOp,t0,x0,warmStart,varargin)
            
            if sum(isnan(x0))>0
                error(getMessage('AcadoMpcOpSolver:Nanxo'))
            end
            
            t  = t0*ones(1,round((mpcOp.horizonLength-obj.stepSize)/obj.stepSize+1)) + (0:obj.stepSize:(mpcOp.horizonLength-obj.stepSize));
            tx = t0*ones(1,round((mpcOp.horizonLength-obj.stepSize)/obj.stepSize+2)) + (0:obj.stepSize:(mpcOp.horizonLength));
            
            fakeTime = (0:obj.stepSize:(mpcOp.horizonLength));
            
            if not(isempty(warmStart))
                InitControl = [fakeTime',warmStart.u'];
                InitState   = [fakeTime',tx',warmStart.x',zeros(size(fakeTime'))];
            else
                
                %InitControl = [fakeTime', zeros(length(fakeTime),mpcOp.system.nu)];
                %InitState   = [fakeTime', zeros(length(fakeTime),mpcOp.system.nx+1)];
                InitControl = fakeTime;
                InitState   = fakeTime;
                
            end
            
            statesList = num2str(1:length(x0),', x0(%d)');
            statesList = ['t0 ',statesList];
            tic
            
            eval(sprintf('out = %s_RUN(%s,InitControl,InitState);',obj.acadoProblemName,statesList));
            %eval(sprintf('out = %s_RUN(%s,InitState,InitControl);',obj.acadoProblemName,statesList));
            ret.problem = 0;
            
            
            
            ret.solverTime = toc;
            
            ret.u_opt  = out.CONTROLS(:,2:end)';
            ret.tu_opt = out.CONTROLS(:,1)'+t0*ones(1,length(out.CONTROLS(:,1)));
            
            %The fist colum is the time, the second is the virtual time
            % and the last one is the term L
            ret.x_opt  =  out.STATES(:,3:end-1)';
            ret.tx_opt =  out.STATES(:,2)';
            
            ret.acadoSolverOutput = out;
            
            
            
            if sum(sum(isnan(ret.x_opt)))>0 || ...
                    max(max(ret.u_opt.*ret.u_opt)) > 10^(2*2) || ...
                    obj.conditionProblem(out)
                ret.problem = 1;
            end
            
         
            
            ret.solverParameters = {out};
            
        end
        
        function sol = faceProblem(obj,mpcController,problematicSol,t,x,varargin)
            
            warning('AcadoMpcOpSolver: solution problem. Facing problem...ty agrain')
            
            warmStart = [];
            
            sol = mpcController.mpcOpSolver.solve(mpcController.mpcOp,t,x,warmStart,mpcController.solverParameters{:});
            
            
        end
        
        
        function initSimulation(obj)
            
            nx = obj.mpcOp.system.nx;
            nu = obj.mpcOp.system.nu;
            T  = obj.mpcOp.horizonLength;
            
            if isa(obj.mpcOp,'CtMpcOp') || isa(obj.mpcOp,'DtMpcOp')
                ct = isa(obj.mpcOp,'CtMpcOp');
            else
                error(getMessage('AcadoMpcOpSolver:CtMpcOp'));
            end
            
            %% Init
            %> BEGIN_ACADO;
            %> acadoSet('problemname', 'acadoCode');
            
            if(obj.displayAcadoCode) disp('BEGIN_ACADO;');end
            BEGIN_ACADO;                                % Always start with "BEGIN_ACADO".
            
            if(obj.displayAcadoCode) disp(sprintf('acadoSet(''problemname'', ''%s''); ',obj.acadoProblemName));end
            eval(sprintf('acadoSet(''problemname'', ''%s''); ',obj.acadoProblemName));
            
            %--------------------------------------------------------------
            %% Differential States
            %
            % DifferentialState t;
            % DifferentialState x1;
            % ...
            % DifferentialState xn;
            % DifferentialState L;
            
            if(obj.displayAcadoCode) disp('DifferentialState t;'); end
            eval('DifferentialState t;');
            
            
            for i = 1:nx
                if(obj.displayAcadoCode) disp(sprintf('DifferentialState x%d;',i)); end
                eval(sprintf('DifferentialState x%d;',i));
            end
            
            if(obj.displayAcadoCode) disp(sprintf('DifferentialState L;')); end
            eval(sprintf('DifferentialState L;'));
            
            %--------------------------------------------------------------
            
            %--------------------------------------------------------------
            %% Control Inputs
            %> Control u1;
            %> ...
            
            for i = 1:nu
                if(obj.displayAcadoCode) disp(sprintf('Control u%d;',i)); end
                eval(sprintf('Control u%d;',i));
            end
            %--------------------------------------------------------------
            
            %--------------------------------------------------------------
            %% Initial Conditions
            %> x01=acado.MexInput
            %> ...
            %> initState = acado.MexInputMatrix;
            %> initInput = acado.MexInputMatrix;
            
            if(obj.displayAcadoCode) disp('t0  = acado.MexInput;'); end
            eval('t0  = acado.MexInput;');
            
            for i = 1:nx
                if(obj.displayAcadoCode) disp(sprintf('x0%d = acado.MexInput;',i)); end
                eval(sprintf('x0%d = acado.MexInput;',i));
            end
            
            if(obj.displayAcadoCode) disp('initInput = acado.MexInputMatrix; '); end
            if(obj.displayAcadoCode) disp('initState = acado.MexInputMatrix; '); end
            initInput = acado.MexInputMatrix;
            
            initState = acado.MexInputMatrix;
            %--------------------------------------------------------------
            
            %--------------------------------------------------------------
            %% Obtain strings of the differential equations
            
            x = sym('x',[nx,1]);
            x = sym(x,'real');
            
            u = sym('u',[nu,1]);
            u = sym(u,'real');
            
            tsym = sym('t','real');
            
            fsym = obj.mpcOp.system.f(tsym,x,u);
            lsym = obj.mpcOp.stageCost(tsym,x,u);
            
            if not(size(fsym,1)==nx)
                error(getMessage('AcadoMpcOpSolver:SizeMismatch'));
            end
            
            if not(isempty(obj.mpcOp.terminalCost))
                msym = obj.mpcOp.terminalCost(tsym,x);
            else
                msym=[];
            end
            %--------------------------------------------------------------
            
            
            if ct
                if(obj.displayAcadoCode) fprintf('f = acado.DifferentialEquation();\n'); end
                f = acado.DifferentialEquation();
            else
                
                if(obj.displayAcadoCode) fprintf('f = acado.DiscretizedDifferentialEquation(%d);\n',obj.stepSize); end
                f = acado.DiscretizedDifferentialEquation(obj.stepSize);
                
            end
            
            %% State equations
            %> f.add(dot(x1) == ...);
            %> ...
            %> f.add(dot(L) == ...); % Dummy variable that integrate the stage cost
            ret = obj.getStateEquations(ct,fsym,lsym);
            if(obj.displayAcadoCode) disp(ret); end
            eval(ret);
            
            %% Optimization problem
            %> ocp = acado.OCP(0, T, intervals);
            %> ocp.minimizeMayerTerm(L + terminalCost);
            %> ocp.subjectTo( f );
            
            if ct
                Tend = T;
                nIntervals = round(T/obj.stepSize);
            else
                Tend = T*obj.stepSize;
                nIntervals = T;
            end
            
            if(obj.displayAcadoCode) disp(sprintf('ocp = acado.OCP(0, %d, %d);',Tend, nIntervals));end
            ocp = acado.OCP(0, Tend, nIntervals); %We have to specify the CONTROL intervals
            
            if obj.acadoMinimizeLSQ
                
                args = '{';
                
                for i =1:length(lsym)-1;
                    args = [args,char(lsym(i)),','];
                end
                
                args = [args,char(lsym(end)),'}'];
                
                if(obj.displayAcadoCode) disp(sprintf('ocp.minimizeLSQ(%s);',args));end
                
                eval(sprintf('ocp.minimizeLSQ(%s);',args));
                
            else
                
                if not(isempty(msym)) && isa(msym,'sym') 
                    if(obj.displayAcadoCode) disp(sprintf('ocp.minimizeMayerTerm(L + %s);',char(msym)));end
                    eval(sprintf('ocp.minimizeMayerTerm(L + %s);',char(msym)));
                else
                    if(obj.displayAcadoCode) disp('ocp.minimizeMayerTerm(L);');end
                    ocp.minimizeMayerTerm(L);
                end
            end
            
            if(obj.displayAcadoCode) disp('ocp.subjectTo( f );  '); end
            
            ocp.subjectTo( f );
            
            %% Initial Constraints
            %> ocp.subjectTo( 'AT_START', x1     ==  x01 );
            %> ...
            
            if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_START'', t    ==  t0 ); ')  ); end
            eval(sprintf('ocp.subjectTo( ''AT_START'', t    ==  t0 ); ') );
            
            %Initial conditions
            for i = 1:nx
                
                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_START'', x%d   ==  x0%d ); ', i,i) ); end
                eval(sprintf('ocp.subjectTo( ''AT_START'', x%d   ==  x0%d ); ', i,i) );
                
            end
            
            
            if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_START'', L    ==  0 ); ')  ); end
            eval(sprintf('ocp.subjectTo( ''AT_START'', L    ==  0 ); ') );
            
            
            %% Stage Constraints
            %> ocp.subjectTo( x1 <=... );
            %> ...
            
            ret = getGeneralSetConstraints(obj,obj.mpcOp.stageConstraints);
            if(obj.displayAcadoCode) disp(ret); end
            eval(ret);
            
            %% Terminal Constraints
            %> ocp.subjectTo( x1 <=... );
            %> ...
            
            
            ret = getGeneralSetConstraints(obj,obj.mpcOp.terminalConstraints,'''AT_END'',');
            if(obj.displayAcadoCode) disp(ret); end
            eval(ret);
            
            %% Optimization algorithm setting
            % algo = acado.OptimizationAlgorithm(ocp);
            % algo.set('KKT_TOLERANCE', 1e-6);
            % algo.initializeDifferentialStates(InitState),
            % algo.initializeControls(initInput),
            % ...
            % END_ACADO
            if(obj.displayAcadoCode) disp(sprintf('algo = acado.OptimizationAlgorithm(ocp);')); end
            
            algo = acado.OptimizationAlgorithm(ocp);
            
            for i = 1:2:length(obj.acadoOptimizationAlgorithmOptions)
                
                optioni = obj.acadoOptimizationAlgorithmOptions{i};
                valuei  = obj.acadoOptimizationAlgorithmOptions{i+1};
                
                if isnumeric(valuei) && obj.displayAcadoCode
                    disp(sprintf('algo.set(''%s'',%d);',optioni,valuei) );
                elseif obj.displayAcadoCode
                    disp(sprintf('algo.set(''%s'',''%s'');',optioni,valuei) );
                end
                
                algo.set(optioni,valuei);
                
            end
            
            if(obj.displayAcadoCode)
                
                disp('algo.initializeDifferentialStates(initState);');
                disp('algo.initializeControls(initInput);');
                disp('END_ACADO;');
                
            end
            
            algo.initializeDifferentialStates(initState);
            algo.initializeControls(initInput);
            
            if(obj.displayAcadoCode) disp('END_ACADO;'); end
            END_ACADO;
        end
        
    
        
        function ret = getStateEquations(obj,ct,fsym,lsym)
            %% State equations
            %> f.add(dot(x1) == ...);
            %> ...
            %> f.add(dot(L) == ...); % Dummy variable that integrate the stage cost
            
            nx = obj.mpcOp.system.nx;
            
            ret = '% State equations';
            
            
            if ct
                ret = sprintf('%s \n f.add(dot(t) == 1);',ret);
            else
                ret = sprintf('%s \n f.add(next(t) == t+1);',ret);
            end
            
            
            for i = 1:nx
                
                if ct
                    ret = sprintf('%s \n f.add(dot(x%d) == %s);',ret, i, char(fsym(i)));
                else
                    ret = sprintf('%s \n f.add(next(x%d) == %s);',ret, i, char(fsym(i)));
                end
            end
            
            if ct
                if isnumeric(lsym) && lsym == 0
                    clsym='0';
                else
                    clsym=char(lsym);
                end
                ret = sprintf('%s \n f.add(dot(L) == %s);',ret, clsym);
            else
                ret = sprintf('%s \n f.add(next(L) == L + %s);',ret, char(lsym*obj.stepSize));
            end
            
        end
        
        function ret = getGeneralSetConstraints(obj,stageConstraints,str)
            
            ret = '';
            for j =1:length(stageConstraints)
                
                con = stageConstraints{j};
                if nargin == 2
                    str = '';
                end
                if strcmp(str,'''AT_END'',')
                    ret = '% GeneralSet TerminalConstraints';
                else
                    ret = '% GeneralSet StageConstraints';
                end
                
                nx = obj.mpcOp.system.nx;
                nu = obj.mpcOp.system.nu;
                
                x = sym('x',[nx,1]);
                x = sym(x,'real');
                
                u = sym('u',[nu,1]);
                u = sym(u,'real');
                
                tsym = sym('t','real');
                
                
                
                
                if strcmp(str,'') && con.nx == nx+ nu
                    
                    fx = con.f([x;u]);
                    
                elseif strcmp(str,'') && con.nx == nx+ nu + 1
                    
                    fx = con.f([tsym;x;u]);
                    
                elseif strcmp(str,'''AT_END'',') && con.nx == nx
                    
                    fx = con.f(x);
                    
                elseif strcmp(str,'''AT_END'',') && con.nx == nx + 1
                    
                    fx = con.f([tsym;x]);
                    
                else
                    if strcmp(str,'')
                        
                        error(getMessage('StageSetDimensionsMismatch'));
                        
                    elseif strcmp(str,'''AT_END'',')
                        
                        error(getMessage('TerminalSetDimensionsMismatch'));
                        
                    end
                end
                
                for i = 1:con.nf
                    
                    ret = sprintf('%s \n ocp.subjectTo( %s %s ); ',ret,str, strcat(char(fx(i)),'<=0'));
                    
                end
                
            end
        end
        
        
    end
    
    
    
end