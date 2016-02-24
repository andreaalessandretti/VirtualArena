classdef InlineTrajectory < Trajectory
    %TRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mypd
        mypdDot
        
    end
    
    methods 
        
        function obj = InlineTrajectory(n,pd,pdDot)
            obj.mypd = pd;
            obj.mypdDot = pdDot;
            obj.n = n;
        end
        
        function v = pd(obj,t)
            v = obj.mypd(t);
        end
        
        function v = pdDot(obj,t)
            v = obj.mypdDot(t);
        end
        
    end
    
end

