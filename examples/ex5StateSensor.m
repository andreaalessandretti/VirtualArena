
classdef ex5StateSensor < Sensor
    methods (Static)
        
        function  measurement = sense(agentId,agent,detectableAgentsList,detectableAgentsIds)
            measurement = {};
            nDetectables = length(detectableAgentsIds);
            for i = 1 : nDetectables
                measurement{i} = detectableAgentsList{i}.x;
            end 
        end
        
    end
end

