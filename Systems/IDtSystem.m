
%%IDtSystem discrete-time system
%
% Example:
%
% Inline implementation of the discrete-time system
%
% x+ = A*x+B*u ,  x(0) = [10;20]
%
% with x \in R^nx and u \in R^nu, in closed-loop with
%
% u = -K*x
%
%
% VA code:
%
%
% A = [1,2;0,1]; B = [0;1];
%
% [K,P,E] = dlqr(A,B,eye(2),100);
%
% sys = IDtSystem('StateEquation',@(k,x,u)A*x+B*u,'nx',2,'nu',1);
%
% sys.controller = IController(@(k,x)-K*x);
%
% sys.initialCondition = [10;20];
%
% va = VirtualArena(sys,'StoppingCriteria'  ,@(k,as)k>30);
%
% ret = va.run();
%
%
% See also DynamicalSystem, DtSystem



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




classdef IDtSystem < DtSystem
    
    properties
        StateEquation
        OutputEquation
    end
    
    methods
        
        
        function obj = IDtSystem (varargin)
            
            obj = obj@DtSystem(varargin{:});
            
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