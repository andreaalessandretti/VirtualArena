classdef MyCtSystem < CtSystem
   
    methods
        
        function obj = MyCtSystem
            obj = obj@CtSystem('nx',2,'nu',1);
        end
        
        function xDot = f(obj,k,x,u)
            xDot = [k*x(1) + u;
                    x(2)*u];
        end
        
        
    end
    
end

