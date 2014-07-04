classdef AcadoMpcOpSolver < MpcOpSolver & InitDeinitObject
%AcadoMpcOpSolver
% 
% Parameters :
%
% AcadoProblemName   (Default 'acadoCode')
% AcadoNIntervals    (Default 10)
% DisplayAcadoCode   (Default 1)
% DiscretizationStep If set != 0 means that the synamic is a discrete time
%                    system. For this case, some ACADO functionalities are
%                    still under development.
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
        
        blackBox =[];
        
    end
    
    methods
        
        function obj = AcadoMpcOpSolver(varargin)
            
            parameterPointer = 1;
            obj.discretizationStep =0.1;
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'DisplayAcadoCode'
                            
                            obj.displayAcadoCode = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                          
                        case 'AcadoMinimizeLSQ'
                            
                            obj.acadoMinimizeLSQ = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                          
                            
                        case 'DiscretizationStep'
                            
                            obj.discretizationStep = varargin{parameterPointer+1};
                            
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
        
        %(obj.mpcOp,x,obj.warmStart,obj.solverParameters{:})
        function ret = solve(obj,mpcOp,x0,warmStart,varargin)
            
            if sum(isnan(x0))>0
                error('NaN initial condition')
            end
            
            if not(strcmp(class(mpcOp),'CtMpcOp') || strcmp(class(mpcOp),'DtMpcOp') || strcmp(class(mpcOp),'LinearizedDtMpcOp'))
                error('Only CtMpcOp and DtMpcOp supported');
            end
            
            %Warm start
            if not(isempty(warmStart))
                %nargin >3 && iscell(varargin) && isstruct(varargin{1})
                % Truncated sequence, without teminal input
                t = 0:obj.discretizationStep:mpcOp.horizonLength-obj.discretizationStep;
                
                %InitControl = varargin{1}.CONTROLS;
                %InitControl(1:end-1,2:end) =  InitControl(2:end,2:end);
                InitControl =  [t',warmStart.u'];
                
            else
                InitState   = zeros(1,mpcOp.system.nx+1);
                InitControl = zeros(1,mpcOp.system.nu+1);
            end
            
            stateList = num2str(1:length(x0),', x0(%d)');
            stateList = stateList(2:end);
            tic
            %if isa(mpcOp,'DtMpcOp') &&
            eval(sprintf('out = %s_RUN(%s,InitControl);',obj.acadoProblemName,stateList));
            %eval(sprintf('out = %s_RUN(%s,InitState,InitControl);',obj.acadoProblemName,stateList));
            
            ret.solverTime = toc;
            
            %The fist colum is the time adn the last of the state is the
            %term L
            ret.u_opt = out.CONTROLS(1:end-1,2:end)';
            ret.x_opt = out.STATES(:,2:end-1)'; 
            ret.acadoSolverOutput = out;
            %ret.problem = not(out.CONVERGENCE_ACHIEVED);
            ret.problem = 0;
            ret.solverParameters = {out};
            
        end
        
        function initSimulations(obj)
            disp('init');
            nx = obj.mpcOp.system.nx;
            nu = obj.mpcOp.system.nu;
            T  = obj.mpcOp.horizonLength;
            
            
            if strcmp(class(obj.mpcOp),'CtMpcOp') || strcmp(class(obj.mpcOp),'DtMpcOp')|| strcmp(class(obj.mpcOp),'LinearizedDtMpcOp')
                ct = strcmp(class(obj.mpcOp),'CtMpcOp');
            else
                error('Only CtMpcOp and DtMpcOp supported');
            end
            %% Init
            %> BEGIN_ACADO;
            %> acadoSet('problemname', 'acadoCode');
            
            if(obj.displayAcadoCode) disp('BEGIN_ACADO;');end
            BEGIN_ACADO;                                % Always start with "BEGIN_ACADO".
            
            if(obj.displayAcadoCode) disp(sprintf('acadoSet(''problemname'', ''%s''); ',obj.acadoProblemName));end
            eval(sprintf('acadoSet(''problemname'', ''%s''); ',obj.acadoProblemName));
            
            %% Differential States
            %> DifferentialState x1;
            %> ...
            if not(isempty(obj.blackBox))
                %In the blackBox the lagtangian cost is specivied by
                %Default as last component of the state vector
                
                for i = 1:nx+1
                    if(obj.displayAcadoCode) disp(sprintf('DifferentialState x%d;',i)); end
                    eval(sprintf('DifferentialState x%d;',i));
                end
            
                else

                for i = 1:nx
                    if(obj.displayAcadoCode) disp(sprintf('DifferentialState x%d;',i)); end
                    eval(sprintf('DifferentialState x%d;',i));
                end

                if(obj.displayAcadoCode) disp(sprintf('DifferentialState L;')); end
                eval(sprintf('DifferentialState L;'));
            end
            
            
            %% Control Inputs
            %> Control u1;
            %> ...
            
            for i = 1:nu
                if(obj.displayAcadoCode) disp(sprintf('Control u%d;',i)); end
                eval(sprintf('Control u%d;',i));
            end
            
            %% Initial Conditions
            %> x01=acado.MexInput
            %> ...
            %> InitState = acado.MexInputMatrix;
            %> initInput = acado.MexInputMatrix;
            
            for i = 1:nx
                if(obj.displayAcadoCode) disp(sprintf('x0%d=acado.MexInput;',i)); end
                eval(sprintf('x0%d=acado.MexInput;',i));
            end
            
            %if(obj.displayAcadoCode) disp('InitState = acado.MexInputMatrix; '); end
            if(obj.displayAcadoCode) disp('initInput = acado.MexInputMatrix; '); end
            
            %InitState = acado.MexInputMatrix;
            initInput = acado.MexInputMatrix;
            
            % Obtain string of the differential equations
            if not(isempty(obj.blackBox))
                x = sym('x',[nx+1,1]);
                x = sym(x,'real');
                
                u = sym('u',[nu,1]);
                u = sym(u,'real');
                
                fsym = obj.mpcOp.system.f(x(1:end-1),u);
                lsym = obj.mpcOp.stageCost(x(1:end-1),u);
                msym = obj.mpcOp.terminalCost(x(1:end-1));
            else
                 
                x = sym('x',[nx,1]);
                x = sym(x,'real');
                
                u = sym('u',[nu,1]);
                u = sym(u,'real');
                
                fsym = obj.mpcOp.system.f(x,u);
                lsym = obj.mpcOp.stageCost(x,u);

                if not(size(fsym,1)==nx)
                    error('The size of the derivative of the x is different from nx');
                end

                if not(isempty(obj.mpcOp.terminalCost))
                    msym = obj.mpcOp.terminalCost(x);
                else
                    msym=[];
                end
            
            end
            
            
            
            if ct
                if(obj.displayAcadoCode) fprintf('f = acado.DifferentialEquation();\n'); end
                f = acado.DifferentialEquation();
            else
                
                    if(obj.displayAcadoCode) fprintf('f = acado.DiscretizedDifferentialEquation(%d);\n',obj.discretizationStep); end
                    f = acado.DiscretizedDifferentialEquation(obj.discretizationStep);
               
            end
            
            %% State equations
            %> f.add(dot(x1) == ...);
            %> ...
            %> f.add(dot(L) == ...); % Dummy variable that integrate the stage cost
            
            if not(isempty(obj.blackBox))
                if(obj.displayAcadoCode) disp(sprintf('f.linkMatlabODE(''%s'');', obj.blackBox)); end
                        eval((sprintf('f.linkMatlabODE(''%s'');', obj.blackBox)));
            else
                
                for i = 1:nx
                    if ct
                        if(obj.displayAcadoCode) disp(sprintf('f.add(dot(x%d) == %s);', i, char(fsym(i)) )); end
                        eval(sprintf('f.add(dot(x%d) == %s);', i, char(fsym(i)) ));
                    else
                        if(obj.displayAcadoCode) disp(sprintf('f.add(next(x%d) == %s);', i, char(fsym(i)) )); end
                        eval(sprintf('f.add(next(x%d) == %s);', i, char(fsym(i)) ));

                    end
                end

                if ct
                       if(obj.displayAcadoCode) disp(sprintf('f.add(dot(L) == %s);', char(lsym))); end
                        eval(sprintf('f.add(dot(L) == %s);', char(lsym)));
                    else
                        if(obj.displayAcadoCode) disp(sprintf('f.add(next(L) == L + %s);', char(lsym*obj.discretizationStep))); end
                        eval(sprintf('f.add(next(L) == L + %s);', char(lsym*obj.discretizationStep)));

                end
            end
            
            
            %% Optimization problem
            %> ocp = acado.OCP(0, T, intervals);
            %> ocp.minimizeMayerTerm(L + terminalCost);
            %> ocp.subjectTo( f );
            
            if ct
                if(obj.displayAcadoCode) disp(sprintf('ocp = acado.OCP(0, %d, %d);',T,round(T/obj.discretizationStep)));end
                ocp = acado.OCP(0, T, round(T/obj.discretizationStep)); %We have to specify the CONTROL intervals
            else
                if(obj.displayAcadoCode) disp(sprintf('ocp = acado.OCP(0, %d, %d);',T*obj.discretizationStep,T) );end
                ocp = acado.OCP(0,T*obj.discretizationStep,T);
            end
            
            if obj.acadoMinimizeLSQ
                
                args = '{';
                for i =1:length(lsym)-1;
                    args = [args,char(lsym(i)),','];
                end
                args = [args,char(lsym(end)),'}'];
                
                if(obj.displayAcadoCode) disp(sprintf('ocp.minimizeLSQ(%s);',args));end
                
                eval(sprintf('ocp.minimizeLSQ(%s);',args));

%                 args = '{';
%                 for i =1:length(msym)-1;
%                     args = [args,char(msym(i)),','];
%                 end
%                 args = [args,char(msym(end)),'}'];
%                 
%                 if(obj.displayAcadoCode)disp(sprintf('ocp.minimizeLSQEndTerm(%s);',args));end
%                 
%                 eval(sprintf('ocp.minimizeLSQEndTerm(%s);',args));
%                 

            else
                if not(isempty(obj.blackBox))
                    
                    if not(isempty(msym))
                    if(obj.displayAcadoCode) disp(sprintf('ocp.minimizeMayerTerm(%s + %s);',char(x(end)),char(msym)));end
                        eval(sprintf('ocp.minimizeMayerTerm(%s + %s);',char(x(end)),char(msym)));
                    else
                        if(obj.displayAcadoCode) disp('ocp.minimizeMayerTerm(L);');end
                        ocp.minimizeMayerTerm(L);
                    end
                else
                    
                    if not(isempty(msym))
                    if(obj.displayAcadoCode) disp(sprintf('ocp.minimizeMayerTerm(L + %s);',char(msym)));end
                        eval(sprintf('ocp.minimizeMayerTerm(L + %s);',char(msym)));
                    else
                        if(obj.displayAcadoCode) disp('ocp.minimizeMayerTerm(L);');end
                        ocp.minimizeMayerTerm(L);
                    end
                end
            
            end
        
            
            
            if(obj.displayAcadoCode) disp('ocp.subjectTo( f );  '); end
            ocp.subjectTo( f );
            
            %% Initial Constraints
            %> ocp.subjectTo( 'AT_START', x1     ==  x01 );
            %> ...
            
            %Initial conditions
            for i = 1:nx
                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_START'', x%d     ==  x0%d ); ', i,i) ); end
                eval(sprintf('ocp.subjectTo( ''AT_START'', x%d     ==  x0%d ); ', i,i) );
            end
            if isempty(obj.blackBox)
            if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_START'', L    ==  0 ); ')  ); end
            eval(sprintf('ocp.subjectTo( ''AT_START'', L    ==  0 ); ') );
            else
            if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_START'', %s    ==  0 ); ',char(x(end)) )  ); end
            eval(sprintf('ocp.subjectTo( ''AT_START'', %s    ==  0 ); ',char(x(end))) );
            end
            %% Stage Constraints
            %> ocp.subjectTo( x1 <=... );
            %> ...
            
            for i =1:length(obj.mpcOp.stageConstraints)
                
                switch class(obj.mpcOp.stageConstraints{i})
                    
                    case 'BoxSet'
                        con = obj.mpcOp.stageConstraints{i};
                        
                 
                               % check if dimentions match
                            if not(nx+nu == con.spaceDimension)

                                error(getMessage('ForcesLinearizedMpcSolver:StageBoxSetDimentionsMismatch'));

                            end
                 
                        
                        for j = 1:nx
                            lbp = find(con.indexesLowerBounds == j);
                            ubp = find(con.indexesUpperBounds == j);
                            
                            if ( not(isempty(lbp)) && not(isempty(ubp)))
                                
                                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( %f <= x%d <= %f ); ', con.lowerBounds(lbp),j ,con.upperBounds(lbp))); end
                                eval(sprintf('ocp.subjectTo( %f <= x%d <= %f ); ', con.lowerBounds(lbp),j ,con.upperBounds(lbp)));
                                
                            elseif not(isempty(lbp))
                                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( %f <= x%d ); ', con.lowerBounds(lbp),j)); end
                                eval(sprintf('ocp.subjectTo( %f <= x%d ); ', con.lowerBounds(lbp),j));
                                
                            elseif not(isempty(ubp))
                                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( x%d <= %f ); ', j ,con.upperBounds(lbp))); end
                                eval(sprintf('ocp.subjectTo( x%d <= %f ); ', j ,con.upperBounds(lbp)));
                            end
                            
                        end
                        
                        for j = 1:nu
                            
                            lbp = find(con.indexesLowerBounds == j+nx);
                            ubp = find(con.indexesUpperBounds == j+nx);
                            
                            if ( not(isempty(lbp)) && not(isempty(ubp)))
                                
                                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( %f <= u%d <= %f ); ', con.lowerBounds(lbp),j ,con.upperBounds(lbp))); end
                                eval(sprintf('ocp.subjectTo( %f <= u%d <= %f ); ', con.lowerBounds(lbp),j ,con.upperBounds(lbp)));
                                
                            elseif not(isempty(lbp))
                                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( %f <= u%d ); ', con.lowerBounds(lbp),j)); end
                                eval(sprintf('ocp.subjectTo( %f <= u%d ); ', con.lowerBounds(lbp),j));
                                
                            elseif not(isempty(ubp))
                                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( u%d <= %f ); ', j ,con.upperBounds(lbp))); end
                                eval(sprintf('ocp.subjectTo( u%d <= %f ); ', j,con.upperBounds(lbp)));
                            end
                        end
                        
                    case 'AcadoSetHack'
                         con = obj.mpcOp.stageConstraints{i};
                         if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo(%s); ', con.str)); end
                         eval(sprintf('ocp.subjectTo(%s); ', con.str) );
                                
                    otherwise
                        disp(getMessage('AcadoMpcOpSolver:StageConstraintNotSupported',class(stageConstratins{i})));
                end
            end
            
            %% Terminal Constraints
            %> ocp.subjectTo( x1 <=... );
            %> ...
            
            for i =1:length(obj.mpcOp.terminalConstraints)
                
                switch class(obj.mpcOp.terminalConstraints{i})
                    
                    case 'BoxSet'
                        con = obj.mpcOp.terminalConstraints{i};
                        
                        % check if dimentions match
                        if not(nx == con.spaceDimension)
                            
                            error(getMessage('ForcesLinearizedMpcSolver:StageBoxSetDimentionsMismatch'));
                            
                        end
                        
                        for j = 1:nx
                            lbp = find(con.indexesLowerBounds == j);
                            ubp = find(con.indexesUpperBounds == j);
                            
                            if ( not(isempty(lbp)) && not(isempty(ubp)))
                                
                                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_END'', %f <= x%d <= %f ); ', con.lowerBounds(lbp),j ,con.upperBounds(lbp))); end
                                eval(sprintf('ocp.subjectTo( ''AT_END'', %f <= x%d <= %f ); ', con.lowerBounds(lbp),j ,con.upperBounds(lbp)));
                                
                            elseif not(isempty(lbp))
                                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_END'', %f <= x%d ); ', con.lowerBounds(lbp),j)); end
                                eval(sprintf('ocp.subjectTo( ''AT_END'', %f <= x%d ); ', con.lowerBounds(lbp),j));
                                
                            elseif not(isempty(ubp))
                                if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_END'', x%d <= %f ); ', j ,con.upperBounds(lbp))); end
                                eval(sprintf('ocp.subjectTo( ''AT_END'', x%d <= %f ); ', j ,con.upperBounds(lbp)));
                            end
                            
                        end
                    case 'NonlinearSet'
                        con = obj.mpcOp.terminalConstraints{i};
                        if con.nf == 1
                            ineq = strcat(char(con.f(x)),'<=0');
                             if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_END'', %s ); ',ineq)); end
                                eval(sprintf('ocp.subjectTo( ''AT_END'', %s ); ',ineq));
                        else
                             disp(getMessage('AcadoMpcOpSolver:StageConstraintNotSupported',class(obj.mpcOp.terminalConstraints{i})));
                        end
                    otherwise
                        if isa(obj.mpcOp.terminalConstraints{i},'NonlinearSet')
                            con = obj.mpcOp.terminalConstraints{i};
                            if con.nf == 1
                                ineq = strcat(char(con.f(x)),'<=0');
                                 if(obj.displayAcadoCode) disp(sprintf('ocp.subjectTo( ''AT_END'', %s ); ',ineq)); end
                                    eval(sprintf('ocp.subjectTo( ''AT_END'', %s ); ',ineq));
                            else
                                 disp(getMessage('AcadoMpcOpSolver:StageConstraintNotSupported',class(obj.mpcOp.terminalConstraints{i})));
                            end
                        else
                            disp(getMessage('AcadoMpcOpSolver:StageConstraintNotSupported',class(obj.mpcOp.terminalConstraints{i})));
                        end
                        end
            end
            
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
                valuei = obj.acadoOptimizationAlgorithmOptions{i+1};
                if isnumeric(valuei) && obj.displayAcadoCode
                    disp(sprintf('algo.set(''%s'',%d);',optioni,valuei) ); 
                elseif obj.displayAcadoCode
                    disp(sprintf('algo.set(''%s'',''%s'');',optioni,valuei) ); 
                end
                algo.set(optioni,valuei);
            end
            
            if(obj.displayAcadoCode) 
                %disp('algo.initializeDifferentialStates(InitState);');
                disp('algo.initializeControls(initInput);');
                disp('END_ACADO;');  
            end
            
            %algo.initializeDifferentialStates(InitState);
            algo.initializeControls(initInput);
            
            if(obj.displayAcadoCode) disp('END_ACADO;'); end
            END_ACADO;
        end
        
         function solution = faceProblem(obj,solution,mpcOp,xim1)
                    solution.solverParameters{1}
                    error('Acado solver problem.')
         end
         function close(obj)
         end
         
    end
    
    
end