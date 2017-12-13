classdef MinMaxKeeper < handle
   
    properties
        minVal = inf;
        maxVal= -inf;
    end
    
    methods 
    
        function add(obj,val)
            obj.addMax(val);
            obj.addMin(val);
        end
        
        function addMax(obj,val)
            obj.maxVal = max(obj.maxVal,val);
        end
        
        function addMin(obj,val)
            obj.minVal = min(obj.minVal,val);
        end
        
        function ret = getMax(obj)
            ret = obj.maxVal;
        end
        function ret = getMin(obj)
            ret = obj.minVal;
        end
    end
    
end