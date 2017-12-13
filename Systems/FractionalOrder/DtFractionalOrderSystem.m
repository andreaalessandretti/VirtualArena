classdef DtFractionalOrderSystem < FractionalOrderSystem
    %
    %   sum_{i=1}^{l} Ai Delta^{alpha_i} x(k+1) = sum_{i=1}^{r} Ai Delta^{beta_i} u(k)
    %
    % DtFractionalOrderSystem(alpha,beta,As,Bs)

    
    methods
        function obj = DtFractionalOrderSystem(alpha,beta,gamma,As,Bs,Gs)
            
            obj = obj@FractionalOrderSystem(alpha,beta,gamma,As,Bs,Gs);
            
        end
        
        function ret = satisfiesAss1(obj)
            
            M = zeros(size(obj.As{1}));
            
            for i=1:length(obj.As)
                M = M + obj.As{i};
            end
            
            ret = rank(M)==length(M);
            
        end
        
        
    end
    
end