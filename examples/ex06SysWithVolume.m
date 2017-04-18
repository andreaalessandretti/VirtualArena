classdef ex06SysWithVolume < CtSystem

    properties
        volume 
    end
    
    methods
        function obj = ex06SysWithVolume(volumeSet)
            obj = obj@CtSystem('nx',3,'nu',3);
            obj.volume = volumeSet;
        end
        function xDot = f(obj,t,x,u,varargin) % x = [p_x,p_y,theta]
            xDot  = u;
        end
    end
    
    
    
end
