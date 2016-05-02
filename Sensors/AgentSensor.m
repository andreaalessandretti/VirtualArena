
classdef AgentSensor < Sensor
%% AgentSensor(funOfAgent)
% AgentSensor senses a function of the neighboring agents.
% e.g.,
% to sense the sate use funOfAgent = @(agent)agent.x
%
% See also Sensor
    
    properties
        funOfAgent
    end
    
    methods 
        function obj = AgentSensor(funOfAgent)
            
            obj.funOfAgent = funOfAgent;
            
        end
        
        function  ret = sense(obj,agentId,agent,detectableAgentsList,detectableAgentsIds)
            
            ret = {};
            
            nDetectables = length(detectableAgentsIds);
            
            for i = 1 : nDetectables
                
                ret{i} = obj.funOfAgent(detectableAgentsList{i});

            end 
        end
        
 
    end
    
end

