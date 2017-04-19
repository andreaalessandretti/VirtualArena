classdef TrackingController_ECC13 < Controller
    
    properties
        epsilon,cdes,cdesDot,K,PinvE
    end
    
    methods
        
        function obj = TrackingController_ECC13(cdes,cdesDot,K,epsilon)
            
            obj = obj@Controller();
            
            obj.cdes      = cdes;
            obj.cdesDot   = cdesDot;
            obj.K       = K;
            obj.epsilon = epsilon;
            
            E   = [[1;0],[-epsilon(2);epsilon(1)]];

            if not(rank(E)==size(E,1))
                error('System not controllable, try a different epsilon');
            end
            
            obj.PinvE = inv(E);
            
        end
        
        function u = computeInput(obj,t,x)
           
            R = [cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
            e = obj.computeError(t,x);
            u = -obj.PinvE*( obj.K*e -R'*obj.cdesDot(t));
            
        end
        
        function e = computeError(obj,t,x)
           
            p = x(1:2);
            R = [cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
            e = R'*(p-obj.cdes(t))+obj.epsilon;
            
        end
        
       
        
    end
    
end