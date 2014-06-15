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
    % Copyright (C) 2012-14 Andrea Alessandretti
    %
    % andrea.alessandretti@{ist.utl.pt, epfl.ch}
    % Automatic Control Laboratory, EPFL, Lausanne, Switzerland.
    % Institute System and Robotics, IST, Lisbon, Portugal.
    % 
    % This program is free software: you can redistribute it and/or modify
    % it under the terms of the GNU General Public License as published by
    % the Free Software Foundation, either version 3 of the License, or
    % (at your option) any later version.
    % 
    % This program is distributed in the hope that it will be useful,
    % but WITHOUT ANY WARRANTY; without even the implied warranty of
    % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    % GNU General Public License for more details.
    % 
    % You should have received a copy of the GNU General Public License
    % along with this program.  If not, see <http://www.gnu.org/licenses/>.

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