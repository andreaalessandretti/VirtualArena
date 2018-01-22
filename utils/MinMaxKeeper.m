classdef MinMaxKeeper < handle
   
    properties
        minVal = inf;
        maxVal= -inf;
    end
    
    methods 
    
        function add(obj,val)
            if length(val)==1
                obj.addMax(val);
                obj.addMin(val);
            else
                obj.addMax(max(val));
                obj.addMin(min(val));
            end
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
        
        function ret = getSpan(obj)
            ret = obj.maxVal-obj.minVal;
        end
        
        function ret = getMin(obj)
            ret = obj.minVal;
        end
    end
    
end