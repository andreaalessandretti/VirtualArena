
classdef MpcController < Controller & InitDeinitObject
    %MpcController
    %
    %   MpcController properties:
    %
    %   mpcOp            - MpcOp problem to be solved at every iteration
    %   mpcOpSolver      - solver used to solve the MpcOp
    %   solverParameters - parameters of the solver MpcOpSolver
    %   log              - see help MpcController.log
    %
    %   MpcController constructor:
    %
    %       va = MpcController(par1,val1,par2,val2,...)
    %
    %   where the parameters are chosen among 'MpcOp', 'MpcOpSolver', or
    %   'WarmStarter', MpcOpSolverParameters
    %
    %   where
    %
    %   WarmStartMode = 1 shifts the previous optimal trajectory and gives
    %   it to MpcOpSolver as initialization to compute the next solution
    %
    %   See also Controller, MpcOp, MpcOpSolver
    
    
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
        mpcOp
        mpcOpSolver
        solverParameters = {};
        
        %  log.computationTime(i) - time to compute u at step i
        %  log.stageCost(i)       - stage cost at step i
        %  log.solverTime(i)      - time to solve the Op at step i
        log
        
        warmStarter;
        auxiliaryLaw;
        runAuxiliaryLaw = 0;
        lastSolution;
        
        initialController = {};
    end
    
    properties(SetAccess = private)
        i = 1
        blockSizeAllocation = 100;
    end
    
    methods
        
        function obj = MpcController(varargin)
            
            obj.warmStarter = ShiftAndHoldWarmStart();
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'InitialController'
                            
                            obj.initialController = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                            
                        case 'MpcOp'
                            
                            obj.mpcOp = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                            
                        case 'MpcOpSolver'
                            
                            obj.mpcOpSolver = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                            
                        case 'MpcOpSolverParameters'
                            
                            obj.solverParameters = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                            
                        case 'WarmStarter'
                            
                            obj.warmStarter  = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'AuxiliaryLaw'
                            
                            obj.auxiliaryLaw = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'RunAuxiliaryLaw'
                            
                            obj.runAuxiliaryLaw = varargin{parameterPointer+1};
                            
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
        
        function u = computeInput(obj,t,x,varargin)
            
            
            if sum(isnan(x))>0
                disp('mmm');
            end
            
            runSolver = 0;
            tic
            if ~isempty(obj.initialController) && t<= obj.initialController{1}
                
                u = obj.initialController{2}.computeInput(t,x);
                
            elseif obj.runAuxiliaryLaw
                
                u = obj.mpcOp.auxiliaryLaw.computeInput(t,x);
                
            else
                
                runSolver = 1;
                
                if not(isempty(obj.lastSolution))
                    warmStart = obj.warmStarter.generateWarmStarts(obj.lastSolution.t,obj.lastSolution);
                else
                    warmStart = [];
                end
                
                if iscell(warmStart) % multiple warm starts
                    for i=1:length(warmStart)
                        ws = warmStart{i};
                        sols{i} = obj.mpcOpSolver.solve(obj.mpcOp,t,x,ws,obj.solverParameters{:});
                    end
                    sol = obj.warmStarter.returnWinningSolution(t,sols);
                    u   = sol.u_opt(:,1);
                else
                    sol = obj.mpcOpSolver.solve(obj.mpcOp,t,x,warmStart,obj.solverParameters{:});
                    
                    if sol.problem
                        sol = obj.mpcOpSolver.faceProblem(obj,sol,t,x,varargin);
                    end
                    
                    u   = sol.u_opt(:,1);
                end
                
            end
            
            obj.appendVectorToLog(toc, obj.i, 'computationTime')
            
            %% Logging
            if runSolver
                
                obj.lastSolution   = sol;
                obj.lastSolution.t = t;
                obj.appendVectorToLog(sol.solverTime ,obj.i,'solverTime');
                
            end
            
            obj.appendVectorToLog(obj.mpcOp.stageCost(t,x,u),obj.i,'stageCost')
            obj.i = obj.i+1;
            
        end
        
        
        function appendVectorToLog(obj,v,i,fildname)
            
            if not(isfield(obj.log,fildname))
                
                obj.log.(fildname) = v;
                
            elseif i>=size(obj.log.(fildname),2)
                
                obj.log.(fildname) =  [obj.log.(fildname),zeros(size(v,1),obj.blockSizeAllocation)];
                
            end
            
            obj.log.(fildname)(:,i) = v;
            
        end
        
        function deinitSimulation(obj)
            obj.lastSolution = [];
        end
        
    end
    
end