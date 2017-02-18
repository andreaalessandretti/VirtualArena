classdef MyShiftAndAppendAuxLawWarmStart < ShiftAndAppendAuxLawWarmStart

   
    methods
        function u = auxiliaryLaw(obj,t,x)
            u = -x*exp(-t);
        end
        
        function xNext = nextX(obj,t,x,u)
            xNext = x*exp(-t)+u;
        end
        
    end
    
end

