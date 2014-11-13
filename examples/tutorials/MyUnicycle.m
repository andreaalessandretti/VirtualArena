
classdef MyUnicycle < CtSystem
    
    methods
        
        function obj =  MyUnicycle()
            
            obj= obj@CtSystem('nx',3,'nu',2);
            
            obj.f = @obj.stateEquation;
            
        end
        
        function xDot = stateEquation(obj,t,x,u)
            
            xDot =  [u(1)*cos(x(3));
                     u(1)*sin(x(3));
                     u(2)          ];
            
        end
    end
    
end