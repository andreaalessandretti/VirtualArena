classdef AgentSensor < Sensor

    methods (Abstract)
       sensingFunction(t,agentId,agent,sensedAgentId,sensedAgent);
    end
    methods 
        
        function ret = sense(obj,t,agentId,agent,detectableAgentsList,detectableAgentsIds)
           
            ret = {};
            for i = 1:length(detectableAgentsIds)
                sensedAgentId  = detectableAgentsIds(i);
                sensedAgent    = detectableAgentsList{i};
                ret{i}         = obj.sensingFunction(t,agentId,agent,sensedAgentId,sensedAgent);
            end
            
        end
        
    end
    
 
    
    
end