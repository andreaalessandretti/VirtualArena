classdef MpcOpSolver < handle
%MpcOpSolver abstract class for and solver of MPC optimization problems
%
%   MpcOpSolver methods:
%
%   solve(obj,MpcOP)                      - executed at every step (see help)
%
%   faceProblem(obj,solution,mpcOp,xim1)  - function run in case of problems 
%                                           (e.g., infeasibility)
%
%   init(obj,MpcOP)                       - function run before using the 
%                                           solver (e.g., memory allocations)
%
%   close(obj)                            - function run after using the solver
%
%   See also MpcOp
    
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
      
    % TODO: redesigned concept.
    % Even if the MpcOp and the system is C.T. the solver will return a
    % discretized version of the optimal trajectories. This parameter
    % specifies with is the associated discretizationStep
    %
    % For the case of purly discrete time sistem this parameter has to be
    % set to 1, which is the defoult value
        discretizationStep = 1
        
        Dl
        
        Hl
        
        Dm
        
        Hm
        
        hessianComputationMethod  = 'Symbolic';
        
        jacobianComputationMethod = 'Symbolic';
        
    end
    
    
    methods (Abstract)
        
        %% solve(obj,MpcOP)
        %   Executed at every iteration, it solves the MpcOp
        %
        %   sol = __.solve(mpcOp,x,warmStart,solverParameters{:});
        %
        %   mpcOp            - MpcOp to solve
        %   x                - initial condition of the MpcOp
        %   warmStart.x      - optimal open loop state prediction at the previous step 
        %   warmStart.u      - optimal open loop control prediction at the previous step 
        %   solverParameters - params specified when creating MpcController
        %
        %   sol.x_opt        - optimal open loop state prediction
        %   sol.u_opt        - optimal open loop control prediction
        solve
        
        faceProblem(mpcController,sol,t,x,varargin)
        
    end
    
    methods
        
        function obj = MpcOpSolver()
        end
        
        
    end
end