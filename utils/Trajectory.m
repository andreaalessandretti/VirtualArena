classdef Trajectory
    %TRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        n
    end
    
    methods (Abstract)
        
    v = pd(obj,t)
    v = pdDot(obj,t)
    
    end

    methods 
        
        function obj = plot(obj,tSpan,varargin)
            pdt = cell2mat( arrayfun(@(t)obj.pd(t),tSpan,'UniformOutput',0));
            plot(pdt(1,:),pdt(2,:),varargin{:});
        end
        
    end
    
end

