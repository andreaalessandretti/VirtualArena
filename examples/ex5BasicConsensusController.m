classdef ex5BasicConsensusController < Controller
    methods
        
        function u = computeInput(obj,t,x,readings)
            nNeigh = length(readings{1});
            u = 0;
            for i =1:nNeigh
                u = u+(readings{1}{i} -x)/nNeigh;
            end
        end
    end
    
end