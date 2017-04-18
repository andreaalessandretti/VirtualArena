classdef Log < handle
    %%Log specifies  what to log during the simulation.
    %
    % These objects are used i) once at the beginning of the simulations 
    % and ii) at the end of every simulation step.
    %
    % Abstract method to implement:
    %
    % vectorToLog = getVectorToLog(t,agent,varargin)
    %
    % that is called as
    %
    % vectorToLog = getVectorToLog(t,sys)      at initialization time t=0;
    % vectorToLog = getVectorToLog(t,sys,u,z)  at the generic tiem t
    %
    % t: time, sys:system, u:input, z: measurement (state or output)
    %
    % Costructor:
    % l = Log(name,par1,val1,par2,val2,...) 
    % where par1, par2, ... are chosen among: 'Initialization', 'Shift'
    %
    % At the beginning of the simulation VA calls getVectorToLog(t,sys),
    % where sys is the generic simulated system, or the value associated 
    % with 'Initialization', if any.
    %
    % The parameter 'Shift' can be used to shift the simulation time where
    % the data is stored.
    %
    % At the end of the simulation, the data can be retrieved using the
    % proposed name. E.g.,
    %
    % va   = VirtualArena(..., 'ExtraLogs',{l} );
    % ret  = va.run();
    % data = ret{nInitialCondition}{nAgent}.logName;
    %
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
        name
        initialization = [];
        shift = 0;
        condition;
        
        deltaStep = [];
        deltaTime = [];
        
        i = 1; %Pointer to the current state to write
    end
    
    methods(Abstract)
        %vectorToLog = getVectorToLog(t,agent)      at initialization time t=0;
        %vectorToLog = getVectorToLog(t,agent,u,z)  at the generic tiem t
        getVectorToLog
        
    end
    
    methods
        function obj = Log(name,varargin)
         
            
            obj.name = name;
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                            
                        case 'Initialization'
                            
                            obj.initialization = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Shift'
                            
                            obj.shift = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Condition'
                            
                            obj.condition = varargin{parameterPointer+1};
                            
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
        
    end
    
end

