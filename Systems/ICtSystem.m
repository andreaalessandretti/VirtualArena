
classdef ICtSystem < CtSystem
%%ICtSystem
% 
% Example:
%
% Inline implementation of the continuous-time system
%
% dot{x} = [0,1;0,0]*x+[0;1]*u ,  x(0) = [1;1]
%
% with x \in R^nx and u \in R^nu, in closed-loop with 
%
% u = -[1,1.7321]*x
%
%
% VA code:
%
%
% sys = ICtSystem(...
%     'StateEquation', @(t,x,u) [0,1;0,0]*x+[0;1]*u,...
%     'nx',2,'nu',1 ...
% );
% 
% sys.initialCondition = [1;1];
% 
% sys.controller = IController(@(t,x)-[1,1.7321]*x);
% 
% va = VirtualArena(sys,'DiscretizationStep', 0.1);
% 
% log = va.run();


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
        StateEquation
        OutputEquation = @(varargin)varargin{2}; % x
    end
    
    methods
        
        
        function obj = ICtSystem (varargin)
            
            obj = obj@CtSystem(varargin{:});
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                            
                        case 'StateEquation'
                            
                            obj.StateEquation = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'OutputEquation'
                            
                            obj.OutputEquation = varargin{parameterPointer+1};
                            
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
        
        function xDot = f(obj,varargin)
            xDot = obj.StateEquation(varargin{:});
        end
        
        function y = h(obj,varargin)
            y = obj.OutputEquation(varargin{:});
        end
          
    end
end