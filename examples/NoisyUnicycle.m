
classdef NoisyUnicycle < Vehicle
    properties
    
    end
    
    methods
 
    end
    
    methods
        function obj =  NoisyUnicycle(varargin)
        
            obj = obj@Vehicle(...
                'nx',3,'nu',2,'ny',3,...
                'PositionSpaceDimension',2,...
                'OutputEquation',@(x,u,w) x(1:3,1)+w...
                ,varargin{:});
            
            obj.f = @(x,u,v)obj.stateEquation(x,u)+v;
           
          
        end
         
    end
    
     methods(Static)
      
               
         function xDot = stateEquation(x,u)
            
            R = [cos(x(3)),-sin(x(3));
                 sin(x(3)), cos(x(3))];

            xDot = [R*[u(1);0];u(2)];
            
         end
         
          function ret = R(x)
               ret = [cos(x(3)),-sin(x(3));
                      sin(x(3)), cos(x(3))];
          end
    end
    
end