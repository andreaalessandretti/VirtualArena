
classdef DtMpcOp < MpcOp
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
        
    end
    
    methods
        
        function obj = DtMpcOp(varargin)
            
            
            if ( nargin == 2 && isa(varargin{1},'CtMpcOp') )
                
                
                ctOP = varargin{1};
                dt = varargin{2};
                
                superClassParameters = DtMpcOp.discretizeCtMpcOp(ctOP,dt);
            else
                
                superClassParameters = varargin;
                
            end
            
            obj = obj@MpcOp(superClassParameters{:});
            
        end
        
    end
    methods(Static)
        function superClassParameters = discretizeCtMpcOp(ctOp,dt)
            
            superClassParameters = ctOp.getParameters();
            % HorizonLength Discretization
            
            index = find(strcmp(superClassParameters, 'HorizonLength'));
            
            hLen = superClassParameters{index +1};
            
            superClassParameters{index +1} = floor(hLen/dt);
            
            
            % Penalization input derivative
            % WARNING:
            % - old implementation
            
            index = find(strcmp(superClassParameters, 'InputDerivative'));
            
            if not(isempty(index)) && not(isempty(superClassParameters{index +1}))  && superClassParameters{index +1}
                
                %% In order to penilize the derivative of the input, we
                % first augment the state as follows
                %
                % xx(i) = [x(i),u(i-1)]
                %
                % with the associated dynamics
                %
                % ff(xx) = [f(x);I]
                %
                % and them we rewrite the agumented stage cost
                %
                % l(x,u,dotU)
                %
                % in the standard form
                %
                % ll(x,u) = l(x(1:nx),u,u-x(nx+(1:nu)))
                %
                % similarly for the terminal cost
                
                % System Discretization
                
                index = find(strcmp(superClassParameters, 'System'));
                ctSys = superClassParameters{index +1};
                dtSys = DtSystem(ctSys,dt);
                
                % Augment the state vector and the dynamic
                
                oldf       = dtSys.f;
                originalNx = dtSys.nx;
                nu         = dtSys.nu;
                dtSys.f    = @(t,x,u)[oldf(x(1:originalNx),u);
                    u];
                
                dtSys.nx = originalNx + nu;
                
                superClassParameters{index +1} = dtSys;
                
                
                % StageCost Discretization and normal form
                
                index = find(strcmp(superClassParameters, 'StageCost'));
                
                ctStCost = superClassParameters{index +1};
                dtStCost = @(t,x,u,dotU) dt*ctStCost(t,x,u,dotU);
                
                dtStCost2 = @(t,x,u) dtStCost(t,...
                    x(1:originalNx),...
                    u,...
                    u-x(originalNx+(1:nu))...
                    );
                
                superClassParameters{index +1} = dtStCost2;
                
                % Terminal cost in normal form
                
                index = find(strcmp(superClassParameters, 'TerminalCost'));
                tCost = superClassParameters{index +1};
                
                superClassParameters{index +1} = @(t,x) tCost(t,x(1:originalNx));
                superClassParameters{index +1} = @(t,x) tCost(t,x(1:originalNx));
                
                
                %% Modify the stage and terminal constratins
                % not that
                %
                % [x(i)  ]     [x(i)       ]
                % [u(i-i)] = A [u(i)       ]
                % [u(i)  ]     [u(i)-u(i-1)]
                %
                % [x(i)       ]     [x(i)  ]
                % [u(i)       ] = T [u(i-1)]
                % [u(i)-u(i-i)]     [u(i)  ]
                %
                % with
                
                T = [eye(originalNx)     , zeros(originalNx,2*nu);
                    zeros(nu,originalNx+nu)    ,eye(nu) ;
                    zeros(nu,originalNx),-eye(nu),eye(nu)];
                
                A = [eye(originalNx)     , zeros(originalNx,2*nu);
                    zeros(nu,originalNx), eye(nu), -eye(nu)     ;
                    zeros(nu,originalNx), eye(nu), zeros(nu,nu) ;];
                
                index = find(strcmp(superClassParameters, 'StageConstraints'));
                
                if not(isempty(index))
                    stageConstratins = superClassParameters{index+1};
                    for i = 1:length(stageConstratins)
                        try
                            stageConstratins{i} = stageConstratins{i}.getAffineTransformation(T,zeros(size(T,1),1));
                        catch e
                            disp('Affine transformation for input derivative failed')
                        end
                        
                    end
                    superClassParameters{index+1} = stageConstratins;
                end
                
                index = find(strcmp(superClassParameters, 'TerminalConstraints'));
                
                % morover
                %
                % [x(N)]  = T [x(N);u(N-1)];
                %
                % with
                
                T = [eye(originalNx), zeros(originalNx,nu)];
                
                if not(isempty(index))
                    terminalConstratins = superClassParameters{index+1};
                    for i = 1:length(terminalConstratins)
                        terminalConstratins{i} = terminalConstratins{i}.getAffineTransformation(T,zeros(size(T,1),1));
                    end
                    superClassParameters{index+1} = terminalConstratins;
                end
                
                
            else
                
                % System Discretization
                
                index = find(strcmp(superClassParameters, 'System'));
                
                if not(isempty(index))
                    ctSys = superClassParameters{index +1};
                    
                    superClassParameters{index +1} = DiscretizedSystem(ctSys,dt);
                end
                
                % StageCost Discretization
                
                index = find(strcmp(superClassParameters, 'StageCost'));
                
                stCost = superClassParameters{index +1};
                if not(isempty(stCost))
                    
                    %% I rewrite extensively this code
                    %if nargin(stCost) <5
                    %   dtStageCost = @(varargin)  dt*stCost(dt*varargin{1},varargin{2:end});
                    %elseif nargin(stCost) >=5%stageCost_jj(k,x_j_i,u_j_i,netReadings,ii);
                    %    dtStageCost = @(varargin)  dt*stCost(dt*varargin{1},varargin{2:4},dt*varargin{5},varargin{6:end});
                    %end
                    % so nargin can detect the number of input
                    
                    if nargin(stCost) ==3
                        dtStageCost = @(k,x,u)  dt*stCost(dt*k,x,u);
                    elseif nargin(stCost) ==4
                        dtStageCost = @(k,x,u,net)  dt*stCost(dt*k,x,u,net);
                    elseif nargin(stCost) ==5
                        dtStageCost = @(k,x,u,net,tau_k)  dt*stCost(dt*k,x,u,net,dt*tau_k);
                    else %nargin(stageCost_jj) ==6
                        dtStageCost = @(varargin)  dt*stCost(dt*varargin{1},varargin{2:4},dt*varargin{5},varargin{6:end});
                        
                    end
                    
                    superClassParameters{index +1} = dtStageCost;
                end
                
                % TerminalCost Discretization
                
                index = find(strcmp(superClassParameters, 'TerminalCost'));
                
                teCost = superClassParameters{index +1};
                
                if not(isempty(teCost))
                    
                    %% I explicit the case 2 and 3 so nargin works on teCost
                    if nargin(teCost) ==2
                        superClassParameters{index +1} = @(k,x)  teCost(dt*k,x);
                    elseif nargin(stCost) ==3
                        superClassParameters{index +1} = @(k,x,net)  teCost(dt*k,x,net);
                    else %nargin(stageCost_jj) ==6
                        superClassParameters{index +1} = @(varargin) teCost(dt*varargin{1},varargin{2:end});
                    end
                end
                
                % StageConstraints Discretization
                
                index = find(strcmp(superClassParameters, 'StageConstraints'));
                stConst = superClassParameters{index +1};
                
                for i = 1:length(stConst)
                    stConsti = stConst{i};
                    if stConsti.nx == ctSys.nx +ctSys.nu +1
                        stConst{i} = GeneralSet(@(x)stConsti.f([dt*x(1);x(2:end)]),stConsti.nx,stConsti.nf);
                    end
                end
                
                superClassParameters{index +1} = stConst;
                
                % TerminalConstraints Discretization
                
                index = find(strcmp(superClassParameters, 'TerminalConstraints'));
                teConst = superClassParameters{index +1};
                
                for i = 1:length(teConst)
                    teConsti = teConst{i};
                    if teConsti.nx == ctSys.nx +ctSys.nu +1
                        teConst{i} = GeneralSet(@(x)teConsti.f([dt*x(1);x(2:end)]),teConsti.nx,teConsti.nf);
                    end
                end
                
                superClassParameters{index +1} = teConst;
                
                % PerformanceConstraints Discretization
                
                index  = find(strcmp(superClassParameters, 'PerformanceConstraints'));
                pConst = superClassParameters{index +1};
                
                for i = 1:length(pConst)
                    
                    pConsti   = pConst{i};
                    pConst{i} = GeneralSet(@(k0,x0,Ji)pConsti.f(dt*k0,x0,Ji),pConsti.nx,pConsti.nf);
                    
                end
                
                superClassParameters{index +1} = pConst;
                
            end
            
            
        end
        
    end
    
    
end