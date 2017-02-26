
classdef LinCtSystem <  CtSystem & LinearizedSystem
    
    methods
        
        function obj = LinCtSystem (sys)
            
            if not( isa(sys,'CtSystem'))
                error ('Only CtSystem allowed');
            end
            
            pars = sys.getParameters();
            obj  = obj@CtSystem(pars{:});
            obj  = obj@LinearizedSystem(sys);
            
        end
        
        
        
        
    end
end