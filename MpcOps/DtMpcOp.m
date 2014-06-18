
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
                
                superClassParameters = ctOP.getParameters();
                
                dt = varargin{2};
                
                % HorizonLength Discretization
                
                index = find(strcmp(superClassParameters, 'HorizonLength'));
                
                hLen = superClassParameters{index +1};
                
                superClassParameters{index +1} = floor(hLen/dt);
                
                  
                % Penalization input derivative
                
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
                    
                    oldf = dtSys.f;
                    originalNx = dtSys.nx;
                    nu = dtSys.nu;
                    dtSys.f = @(x,u)[oldf(x(1:originalNx),u);
                                     u];
                    
                    dtSys.nx = originalNx + nu;
                    
                    superClassParameters{index +1} = dtSys;
                    
                    
                    % StageCost Discretization and normal form

                    index = find(strcmp(superClassParameters, 'StageCost'));

                    ctStCost = superClassParameters{index +1};
                    dtStCost = @(x,u,dotU) dt*ctStCost(x,u,dotU); 
                    
                    dtStCost2 = @(x,u) dtStCost(...
                        x(1:originalNx),...
                        u,...
                        u-x(originalNx+(1:nu))...
                        );
                    
                    superClassParameters{index +1} = dtStCost2;

                    % Terminal cost in normal form

                    index = find(strcmp(superClassParameters, 'TerminalCost'));
                    tCost = superClassParameters{index +1};
                    
                    superClassParameters{index +1} = @(x) tCost(x(1:originalNx));
                superClassParameters{index +1} = @(x) tCost(x(1:originalNx));
                
                    
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

                    ctSys = superClassParameters{index +1};

                    superClassParameters{index +1} = DtSystem(ctSys,dt);


                    % StageCost Discretization

                    index = find(strcmp(superClassParameters, 'StageCost'));

                    stCost = superClassParameters{index +1};

                    superClassParameters{index +1} = @(x,u) dt*stCost(x,u);

              
                end
                
                
            else
                
                superClassParameters = varargin;
                
            end
            
            obj = obj@MpcOp(superClassParameters{:});
            
        end
        
    end
end