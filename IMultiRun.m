
classdef IMultiRun < MultiRun
    
    properties
        
        
        myInitFun %@(VirtualArenaObj,i)
        
        
    end
    
   
    
    methods
        
        function obj = IMultiRun(n,myInitFun)
            obj = obj@MultiRun(n);
            obj.myInitFun = myInitFun;
            
        end
        
        function initFun(obj,va,i)
            obj.myInitFun(va,i);
        end
        
    end
end