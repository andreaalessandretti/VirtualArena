
classdef DiscretizedMpcOp < DtMpcOp
    %DtMpcOP Discrete-time Model Predictive Control Optimization Problem
    %
    %   See help MpcOP for more info
    %
    %   See also MpcOP, CtMpcOp
    
    
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
        originalCtMpcOp
        dtDiscretization
    end
    
    methods
        
        function obj = DiscretizedMpcOp(ctOP,dt)
            
            ctMpcOpParams = ctOP.getParameters();
            
            obj = obj@DtMpcOp(ctMpcOpParams{:});
            
            obj.originalCtMpcOp = ctOP;
            obj.dtDiscretization = dt;
            
            obj.horizonLength = floor(obj.originalCtMpcOp.horizonLength/dt);
            
            obj.system = DiscretizedSystem(obj.originalCtMpcOp.system,dt);
            
            
            % StageConstraints Discretization
            stConst = obj.originalCtMpcOp.stageConstraints;
            ctSys = obj.originalCtMpcOp.system;
            for i = 1:length(stConst)
                stConsti = stConst{i};
                if stConsti.nx == ctSys.nx +ctSys.nu +1
                    stConst{i} = GeneralSet(@(x)stConsti.f([dt*x(1);x(2:end)]),stConsti.nx,stConsti.nf);
                end
            end
            
            obj.stageConstraints = stConst;
            
            
            
            % TerminalConstraints Discretization
            
            teConst =  obj.originalCtMpcOp.terminalConstraints;
            
            for i = 1:length(teConst)
                teConsti = teConst{i};
                if teConsti.nx == ctSys.nx +ctSys.nu +1
                    teConst{i} = GeneralSet(@(x)teConsti.f([dt*x(1);x(2:end)]),teConsti.nx,teConsti.nf);
                end
            end
            
            obj.terminalConstraints = teConst;
            
            
            
            
            
            
            
            % PerformanceConstraints Discretization
            
            pConst = obj.originalCtMpcOp.performanceConstraints;
            
            for i = 1:length(pConst)
                
                pConsti   = pConst{i};
                pConst{i} = GeneralSet(@(k0,x0,Ji)pConsti.f(dt*k0,x0,Ji),pConsti.nx,pConsti.nf);
                
            end
            
            obj.performanceConstraints = pConst;
            
            
        end
        
        
        function l = stageCost(obj,k,x,u,varargin)
            
            dt = obj.dtDiscretization;
            if nargin ==4
                l = dt*obj.originalCtMpcOp.stageCost(dt*k,x,u);
            elseif nargin ==5
                net  = varargin{1};
                l = dt*obj.originalCtMpcOp.stageCost(dt*k,x,u,net);
            elseif nargin ==6
                net = varargin{1};
                tau_k = varargin{2};
                l = dt*stCost(dt*k,x,u,net,dt*tau_k);
            else %nargin(stageCost_jj) ==6
                l = dt*obj.originalCtMpcOp.stageCost(dt*k,x,u,varargin{1},dt*varargin{2},varargin{3:end});
                
            end
            
        end
        
        function m = terminalCost(obj,k,x,varargin)
            dt = obj.dtDiscretization;
            %% I explicit the case 2 and 3 so nargin works on teCost
            if nargin ==3
                m =  obj.originalCtMpcOp.terminalCost(dt*k,x);
            else
                m = obj.originalCtMpcOp.terminalCost(dt*k,x,varargin{:});
            end
        end
        
    end
    
end