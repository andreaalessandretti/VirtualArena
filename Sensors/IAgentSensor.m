classdef IAgentSensor < AgentSensor
%
% Example: sensor that measure the range to the detected agent 
%
% s = IAgentSensor( @(t,agentId,agent,sensedAgentId,sensedAgent)norm(agent.x-sensedAgent.x) ) 
%
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