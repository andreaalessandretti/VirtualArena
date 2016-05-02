
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
        
        function  ret = sense(obj, agentId,agent,detectableAgentsList,detectableAgentsIds)
            ret = {};
            nDetectables = length(detectableAgentsIds);
            
            
            for i = 1 : nDetectables
                
                if isempty(obj.lastReadings) || length(obj.lastReadings)<agentId
                    ny = length(obj.fOfAgent(detectableAgentsList{i}));
                    obj.firstComputationOfDerivative{agentId}{i} = 1;
                    ret{i} = zeros(ny,1);
                    obj.lastReadings{agentId}{i}=zeros(ny,1);
                else
                    if obj.firstComputationOfDerivative{agentId}{i} 
                        ny = length(obj.fOfAgent(detectableAgentsList{i}));
                        obj.firstComputationOfDerivative{agentId}{i} = 0;
                        ret{i} = zeros(ny,1);
                    else
                        ret{i} = (obj.fOfAgent(detectableAgentsList{i})-obj.lastReadings{agentId}{i})/obj.dt;
                    end
                    
                    obj.lastReadings{agentId}{i}=obj.fOfAgent(detectableAgentsList{i});
                end
                

            end 
            
        end
        
 
    end
    
end

