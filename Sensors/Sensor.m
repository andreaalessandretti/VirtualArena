%Sensor
%
%   Sensor abstract method:
%
%   measuraments = sense(agentId,agent,detectableAgentsList,detectableAgentsIds)
%
%   the agent 'agent' with id 'agentId' computes measuraments of the agents
%   'detectableAgentsList' with associated id 'detectableAgentsIds'
%
%   the output 'measurament' is a cell n elements, where n is the number of
%   detectable agents, where measurament{i} is the measurament of agent
%   detectableAgentsIds(i)
%

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

classdef Sensor < handle
    
    properties
        
    end
    
    methods(Abstract)
        
        % The sensor of agent agentId senses the agents
        % detectableAgents==1 of the list agentsList
        sense(obj,agentId,agentsList,detectableAgents);
        
    end
    
end