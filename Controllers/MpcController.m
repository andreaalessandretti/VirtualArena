
classdef MpcController < Controller
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
    %   'SolverParameters'
    %
    %   See also Controller, MpcOp, MpcOpSolver
    
    % This file is part of VirtualArena.
    %
    % Copyright (C) 2012-14 Andrea Alessandretti
    %
    % andrea.alessandretti@{ist.utl.pt, epfl.ch}
    % Automatic Control Laboratory, EPFL, Lausanne, Switzerland.
    % Institute System and Robotics, IST, Lisbon, Portugal.
    %
    % This program is free software: you can redistribute it and/or modify
    % it under the terms of the GNU General Public License as published by
    % the Free Software Foundation, either version 3 of the License, or
    % (at your option) any later version.
    %
    % This program is distributed in the hope that it will be useful,
    % but WITHOUT ANY WARRANTY; without even the implied warranty of
    % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    % GNU General Public License for more details.
    %
    % You should have received a copy of the GNU General Public License
    % along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    properties
        mpcOp
        mpcOpSolver
        solverParameters = {};
        
        %  log.computationTime(i) - time to compute u at step i
        %  log.stageCost(i)       - stage cost at step i
        %  log.solverTime(i)      - time to solve the Op at step i
        log
    end
    
    properties(SetAccess = private)
        i = 1
        blockSizeAllocation = 100;
        lastSolution = {};
    end
    
    methods
        
        function obj = MpcController(varargin)
            
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'MpcOp'
                            
                            obj.mpcOp = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                            
                        case 'MpcOpSolver'
                            
                            obj.mpcOpSolver = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                            
                        case 'MpcOpSolverParameters'
                            
                            obj.solverParameters = varargin{parameterPointer+1};
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
        
        function u = computeInput(obj,x,varargin)
            
            tic
            
            sol = obj.mpcOpSolver.solve(obj.mpcOp,x,obj.solverParameters{:});
            
            obj.appendVectorToLog(toc                     ,obj.i,'computationTime')
            
            u = sol.u_opt(:,1);
            
            obj.appendVectorToLog(obj.mpcOp.stageCost(x,u),obj.i,'stageCost')
            obj.appendVectorToLog(sol.solverTime          ,obj.i,'solverTime')
            obj.lastSolution = sol;
            
            
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
        
        
        
    end
    
end