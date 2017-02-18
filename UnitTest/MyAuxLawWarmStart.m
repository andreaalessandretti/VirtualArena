classdef MyAuxLawWarmStart < AuxLawWarmStart
   
    methods
        
        function obj = MyAuxLawWarmStart
            obj = obj@AuxLawWarmStart(1);
        end
        
        function u = auxiliaryLaw(obj,k,x)
            u = k*x;
        end
        
        function xNext = nextX(obj,k,x,u)
            xNext = k*x+u;
        end
        
    end
    
end

