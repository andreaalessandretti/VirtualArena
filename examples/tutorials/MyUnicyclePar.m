
classdef MyUnicyclePar < CtSystem
    
    properties
        k
    end
    
    methods
        
        function obj =  MyUnicyclePar(k)
            
            obj= obj@CtSystem('nx',3,'nu',2);
            
            obj.f = @obj.stateEquation;
            
            obj.k = k;
            
        end
        
        function xDot = stateEquation(obj,t,x,u)
            
            xDot =  [u(1)*cos(x(3));
                     u(1)*sin(x(3));
                     u(2)*obj.k    ];
            
        end
    end
    
end