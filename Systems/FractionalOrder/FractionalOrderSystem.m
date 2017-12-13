classdef FractionalOrderSystem < handle & InitDeinitObject
% 
%   sum_{i=1}^{l} Ai D^{alpha_i} x = sum_{i=1}^{r} Ai D^{beta_i} u
%
%   obj = FractionalOrderSystem(alpha,beta,As,Bs)
%
    properties
        alpha
        beta
        gamma
        As
        Bs
        Gs
        nx
        nu
    end
    
   methods
       function obj = FractionalOrderSystem(alpha,beta,gamma,As,Bs,Gs)
           if not(length(alpha)==length(As))
                error('required: length(alpha)==length(As)')
           end
           
           if not(length(gamma)==length(Gs))
                error('required: length(gamma)==length(Gs)')
           end
           
           if not(length(beta)==length(Bs))
                error('required: length(beta)==length(Bs)')
           end
           
           obj.alpha = alpha;
           obj.beta  = beta;
           obj.gamma = gamma;
           
           obj.As    = As;
           obj.Bs    = Bs;
           obj.Gs    = Gs;
           
           obj.nx    = size(obj.As{1},1);
           obj.nu    = size(obj.Bs{1},2);
           
       end
       
   end
   
   methods (Static)
   
       function  ret = c_alpha_j(alpha,j)
           %% ret = c_alpha_j(alpha,j) - c_j^alpha
%            if j==0
%                
%                ret  = 0;
%                return
%            end
%            
%            ret = 1;
%            
%            for ii = 0:j-1
%            
%                ret = ret*(alpha-ii)/(ii+1);
%            
%            end
%            
%            ret = ret*(-1)^(-j);
           ret = (-1)^(j)*gamma(alpha+1)/(gamma(alpha-j+1)*factorial(j));
       end
       
   end
   
    
end