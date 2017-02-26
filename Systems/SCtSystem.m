
classdef SCtSystem <  CtSystem & SymbolizedSystem
    
    methods
        
        
        function xDot = f(obj,t,x,u)
            
            xDot = feval(obj.nameFfile,t,x,u);

        end
        
        function y = h(obj,t,x,u)
            
            y = feval(obj.nameHfile,t,x,u);
        
        end
        
        function obj = SCtSystem (sys)
            
            if not( isa(sys,'CtSystem'))
                error ('Only CtSystem allowed');
            end
            
            pars = sys.getParameters();
            obj  = obj@CtSystem(pars{:});
            obj  = obj@SymbolizedSystem(sys);
            
        end
        
        
        
    end
end