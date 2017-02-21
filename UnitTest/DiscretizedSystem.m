

classdef DiscretizedSystem < DtSystem
    
    properties
        OriginalCtSystem;
        Integrator;
        Dt
    end
    
    methods
        
        function obj = DiscretizedSystem (sys,dt,varargin)
            
            
            superClassParameters = sys.getParameters();
            
            obj = obj@DtSystem(superClassParameters{:});
            
            obj.OriginalCtSystem = sys;
            
            obj.Dt = dt;
            
            if nargin == 3
                obj.Integrator = varargin{1};
            else
                obj.Integrator = RK4();
            end
            
        end
        
        function y = h(obj,varargin)
            y = obj.OriginalCtSystem.h(varargin{:});
        end
        
        function xNext = f(obj,varargin)
            dt = obj.Dt;
            k = varargin{1};
            if length(varargin)==3
                
                x = varargin{2};
                u = varargin{3};
                
                xNext = RK4.integrate(@(y)obj.OriginalCtSystem.f(dt*k,y,u),x,dt);
                
            elseif length(varargin)==4
                xc = varargin{2};
                uSysCon = varargin{3};
                x = varargin{4};
                xNext = RK4.integrate(@(y)obj.OriginalCtSystem.f(dt*k,y,uSysCon,x),xc,dt);
                
            elseif length(varargin)==6
                
                x = varargin{2};
                u = varargin{3};
                netReadings = varargin{4};
                t_h = varargin{5};
                x0 = varargin{6};
                
                
                xNext =  RK4.integrate(@(y)obj.OriginalCtSystem.f(dt*k,y,u,netReadings,t_h,x0),x,dt);
                
            else
                error('Too many input for f(.)');
            end
            
        end
        
        
    end
end