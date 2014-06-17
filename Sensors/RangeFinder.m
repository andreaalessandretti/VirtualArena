classdef RangeFinder < Sensor
%RangeFinder 
%
% Given a set of angles and a maximum distance RangeFinder senses the
% vehicles around the agent returning for each angle the distance from the
% vehicles within the specified maximum range.
%
% RangeFinder:
%
% s = RangeFinder(angles);
% s = RangeFinder(angles,maxDistance);
%
% The size of the vehicles are determined by the proper occupancy of the
% class Vehicle. See help Vehicle.
%
% Demo: exRangeFinder
%
% See also Sensor, Vehicle
  
 
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
       angles;
       range = Inf;
    end
    methods 
        
        function obj = RangeFinder(angles,varargin)
            
            obj.angles = angles;
            if nargin>1
                obj.range = varargin{1};
            end
            
        end
        
        function ret = sense(obj,agentId,agentsList,detectableAgents)
            
            agent = agentsList{agentId};
            indexesDetectable = 1:length(detectableAgents);
            indexesDetectable = indexesDetectable(logical(detectableAgents));
            
            
            % R from body to inertial frame
             R = [cos(agent.x(3)),-sin(agent.x(3));
                 sin(agent.x(3)), cos(agent.x(3))];
             
            ret = zeros(length(obj.angles),1);
            for i = 1:length(obj.angles)
                d = R*[cos(obj.angles(i));
                       sin(obj.angles(i))];
                  
                L = -Inf;
                U = Inf;
                for j = indexesDetectable
                    agentj = agentsList{j};
                    set = agentj.occupancy;
                    if not(isempty(set))
                        setShifted = set + agentj.x(1:2);
                        [Lj,Uj] = findClosestIntersectionWithAPolytope(agent.x(1:2),d,setShifted.A,setShifted.b);
                        if Lj<Uj
                            U = min(U,Uj);
                            L = max(L,Lj);
                        end
                    end
                end
                
                    if norm(L)<norm(U)
                         alpha = L;
                    else
                         alpha = U;
                    end
                    
                 ret(i)=alpha;
                
            end
            
            % Remove backward observation (negative values)
            ret(ret<0)= Inf(size( ret(ret<0)));
            
            ret(ret>obj.range) = Inf(size( ret(ret>obj.range)  ) );
            
        end
       
    end
    
end