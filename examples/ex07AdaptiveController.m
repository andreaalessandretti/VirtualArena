classdef ex07AdaptiveController < CtSystem
   
    properties
        L = [1;1];
    end
   
   methods
        % 'xc'  state of the controller
        % 'x'   state of system
        %
        % uSysCon = [u; uCon]
        % 'u' is sent to the system
        % 'uCon' is an extra input that can be used by the controller in
        %        the function f(.), e.g., to drive the state of the
        %        controller.
        %
        %
        
        function uSysCon = h(obj,t,xc,x)
            uSysCon = -xc(2) -x;
        end
        
        function xcDot = f(obj,t,xc,uSysCon,x)
            
            % xDot = -x + p + u
            % pDot = 0
            % xc = [x,p]
            % xcDot = [0,1;0,0]*xc + [1;0]*u

            u = uSysCon(1);
            
            A = [0,1;
                 0,0];
            
            B =  [1;0];
            
            C = [1,0];
            
            xcDot = A*xc + B*u + obj.L*(x-C*xc);
            
        end
        
         %nextXcs{ia} = obj.systemsList{ia}.controller.updateState(timeInfo,nextXc,uSysCon,controllerFParams{:});
        
    end
    
end