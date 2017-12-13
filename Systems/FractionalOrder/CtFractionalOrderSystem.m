classdef CtFractionalOrderSystem < FractionalOrderSystem
% 
%   sum_{i=1}^{l} Ai D^{alpha_i} x = sum_{i=1}^{r} Ai D^{beta_i} u
%
%   obj = CtFractionalOrderSystem(alpha,beta,As,Bs)
%
    
   methods
       function obj = CtFractionalOrderSystem(alpha,beta,As,Bs)
           
           obj = obj@FractionalOrderSystem(alpha,beta,As,Bs);
              
       end
       
   end
    
end