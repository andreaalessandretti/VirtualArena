classdef IAgentSensor < AgentSensor
    
    properties
       sensingFnc; %(t,agentId,agent,sensedAgentId,sensedAgent);
    end
    methods 
        
        function obj = IAgentSensor(sensingFunction)
            
                obj.sensingFnc = sensingFunction;
                
        end
        
        function ret = sensingFunction(obj,t,agentId,agent,sensedAgentId,sensedAgent)
            ret = obj.sensingFnc(t,agentId,agent,sensedAgentId,sensedAgent);
        end
        
        
    end
    
 
    
    
end