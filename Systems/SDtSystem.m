
classdef SDtSystem <  DtSystem & SymbolizedSystem
    
    methods
        
        function obj = SDtSystem(sys)
            
            if not( isa(sys,'DtSystem'))
                error ('Only DtSystem allowed');
            end
            
            pars = sys.getParameters();
            obj  = obj@DtSystem(pars{:});
            obj  = obj@SymbolizedSystem(sys);
            
        end
        
        function xDot = f(obj,t,x,u)
            
            xDot = feval(obj.nameFfile,t,x,u);

        end
        
        function y = h(obj,t,x,u)
            
            y = feval(obj.nameHfile,t,x,u);
        
        end
        
        
    end
end