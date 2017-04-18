classdef MyCtSystem < CtSystem & LinearizedSystem
   
    methods
        
        function obj = MyCtSystem
            obj = obj@CtSystem('nx',2,'nu',1);
            obj = obj@LinearizedSystem();
        end
        
        function xDot = f(obj,k,x,u)
            xDot = [k*x(1) + u;
                    x(2)*u];
        end
        
        
    end
    
end

