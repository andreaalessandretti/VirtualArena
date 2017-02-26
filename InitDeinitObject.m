classdef InitDeinitObject < handle
    %%InitDeinitObject object that needs some ini/deinitialization
    %
    %   Controller methods:
    %
    %   initSimulations   - see help InitDeinitObject.initSimulations
    %   initSimulation    - see help InitDeinitObject.initSimulation
    %   deinitSimulations - see help InitDeinitObject.deinitSimulations
    %   deinitSimulation  - see help InitDeinitObject.deinitSimulation
    
 
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
    

    
    methods
        
        %%initSimulations system initialization
        %   this function is called by VirtualArena at the begining the
        %   simulation. It is called once for the case of simulations from
        %   multiple initial conditions.
        %   
        %   see also DynamicalSystem.initSimulation
        function initSimulations(obj)
            
            obj.executeOnFieldsIfInitDeinitObject('initSimulations');
        end
        
        %%initSimulation system initialization (every initial condition)
        %   this function is called by VirtualArena at the begining the
        %   simulation. It is called at the beginning of every simulation 
        %   for the case of multiple initial conditions.
        %   
        %   see also DynamicalSystem.initSimulations
        function initSimulation(obj)
            
            obj.executeOnFieldsIfInitDeinitObject('initSimulation');
            
        end
        
        %%deinitSimulations system stop
        %   this function is called by VirtualArena at the end the
        %   simulation. It is called once for the case of simulations from
        %   multiple initial conditions
        %   
        %   see also DynamicalSystem.initSimulation
        function deinitSimulations(obj)
            
             obj.executeOnFieldsIfInitDeinitObject('deinitSimulations');
             
        end
        
        %%deinitSimulation system stop (every initial condition)
        %   this function is called by VirtualArena at the begining the
        %   simulation. It is called at the end of every simulation 
        %   for the case of multiple initial conditions.
        %   
        %   see also DynamicalSystem.initSimulations
        function deinitSimulation(obj)
            
            obj.executeOnFieldsIfInitDeinitObject('deinitSimulation');
            
        end
        
        function executeOnFieldsIfInitDeinitObject(obj,functionName)
            filedsList = fields(obj);
            for i = 1:length(filedsList)
                if isa(obj.(char(filedsList(i))),'InitDeinitObject') && not(isa(obj.(char(filedsList(i))),'NoInitDeinitObject'))
                    %char(filedsList(i))
                    obj.(char(filedsList(i))).(functionName);
                end
            end
        end
    end
    
    
end