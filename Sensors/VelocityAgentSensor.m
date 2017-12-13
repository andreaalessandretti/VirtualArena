
classdef VelocityAgentSensor < Sensor
%% VelocityAgentSensor(dt,funOfAgent)
% VelocityAgentSensor senses (estimate) the derivative of a function of 
% the neighboring agents.
%
% e.g.,
% to sense the derivative of the state use funOfAgent = @(agent)agent.x
%
% Note: VelocityAgentSensor approximates the derivative using two consecutive
%       measurements. Therefore, the smaller dt is, the better such estimate
%       (and the performance of the controller) gets.
%
% See also Sensor

    properties

        lastReadings;
        firstComputationOfDerivative;
        fOfAgent;
        dt;
        
    end
    
    methods
        
        function obj = VelocityAgentSensor(dt,fOfAgent)
            obj.dt       = dt;
            obj.fOfAgent = fOfAgent;
        end
        
        function  ret = sense(obj,t, agentId,agent,detectableAgentsList,detectableAgentsIds)
            
            ret = {};
            
            nDetectables = length(detectableAgentsIds);
            
            for i = 1 : nDetectables
                
                var = obj.fOfAgent(detectableAgentsList{i}); % Variable to differentiate
                nVar = length(var);
                if isempty(obj.lastReadings) || length(obj.lastReadings)<agentId
                    
                    obj.firstComputationOfDerivative{agentId}{i} = 1;
                    
                    ret{i} = zeros(nVar,1);
                    
                    obj.lastReadings{agentId}{i}=var;
                    
                else
                    
                    if obj.firstComputationOfDerivative{agentId}{i} 
                        
                        obj.firstComputationOfDerivative{agentId}{i} = 0;
                        
                        ret{i} = zeros(nVar,1);
                        
                    else
                        ret{i} = (var-obj.lastReadings{agentId}{i})/obj.dt;
                    end
                    
                    obj.lastReadings{agentId}{i}=var;
                    
                end
                

            end 
            
        end
        
 
    end
    
end

