
classdef ControllerStateSensor < Sensor
    methods (Static)
        
        function  measurement = sense(agentId,agent,detectableAgentsList,detectableAgentsIds)
            measurement = {};
            nDetectables = length(detectableAgentsIds);
            for i = 1 : nDetectables
                measurement{i} = detectableAgentsList{i}.controller.x(1);
            end 
        end
        
    end
end

