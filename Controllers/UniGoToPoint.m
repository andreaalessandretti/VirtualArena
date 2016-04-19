
classdef UniGoToPoint < Controller
    %UniGoToPoint state-feedback go-to-position Controller for Unicycle model
    %
    % controller = UniGoToPoint (point)
    %
    % point - desired 2d position
    %
    % See also Controller
    
    properties
        point;  
    end
    
    methods
        
        function obj = UniGoToPoint(point)
            
            obj.point = point;
            
        end
        
        function u = computeInput(obj,t,x)
            
            R = [cos(x(3)),-sin(x(3));
                 sin(x(3)), cos(x(3))];      % Body to inertia rotation matrix
            
            err = R'*(obj.point - x(1:2));   % Position error in body frame
            
            thetaErr = atan2(err(2),err(1)); % Angle error
            
            u = [norm(obj.point-x(1:2));
                 thetaErr];
            
        end
        
    end
end