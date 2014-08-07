
classdef CtSystem < GeneralSystem
    %CtSystem Continuous Time System
    %
    %   See help GeneralSystem
    %
    %   CtSystem methods:
    %   CtSystem          - constructor sys = CtSystem(par1,val1,par2,val2, ...)
    %                       see GeneralSystem for the explanation of the
    %                       parameters and values.
    %   getStateTrajectory - Compute a solution
    %   changeOfCoordinate - Perform a change of state and/or input coordinate
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
    
    
    methods
        
        
        function obj = CtSystem (varargin)
            
            obj = obj@GeneralSystem(varargin{:});
            
        end
        
        
        
        function x = getStateTrajectory(obj,t0,x0,u,dt)
            %getStateTrajectory Compute a solution
            %
            % Returns the state trajectory with the form [x0,x1,...]
            % obtained by solving recursively
            %
            % x(:,i+1) = RK4.integrate(@(y)obj.f(t,y,u(:,i)),x(:,i),dt); 
            %     t = t + dt;
            % Calling the function:
            %
            % sys.getStateTrajectory(x0,u,dt)
            
            x = zeros(length(x0),size(u,2)+1);
            x(:,1) = x0;
            t = t0;
            for i =1:size(u,2)
                 x(:,i+1) = RK4.integrate(@(y)obj.f(t,y,u(:,i)),x(:,i),dt); 
                 t = t + dt;
            end
            
            if sum(sum(isnan(x)))>0 || sum(sum(isinf(x)))>0
                error('The solution contains inf or nan elements');
            end
            
        end
        
        
        
        
        
    end
end