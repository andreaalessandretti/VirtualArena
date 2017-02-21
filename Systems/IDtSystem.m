
%%IDtSystem discrete-time system
%
% Consider a discrete-time dynamical model described by
%
% x+ = f(t,x,u)
% y  = h(t,x)/h(t,x,u)
%
% where
%
% x is an nx-dimensional vector
% u is an nu-dimensional vector
% y is an ny-dimensional vector
%
% In VA such system is defined as follows
%
% sys = CtSystem(par1,val1,par2,val2,...)
%
% where the parameters are chosen among
%
% 'StateEquation', 'nx', 'nu', 'OutputEquation', 'ny' ,
% 'InitialCondition', 'Controller','StateObserver'
%
% Often, it is also possible to set the parameters after the creation
% of the object, e.g.,
%
% sys.controller = mycontroller;
% sys.stateObserver = myobserver;
% sys.initialCondition = [1;1];
%
% Some methods provided by this function are the following:
%
%   getStateTrajectory - Compute a solution of the system.
%                        See help CtSystem.getStateTrajectory.
%   changeOfCoordinate - Perform a change of state and/or input
%                        coordinate.
%                        See help CtSystem.changeOfCoordinate.
%
% See also GeneralSystem, DtSystem



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