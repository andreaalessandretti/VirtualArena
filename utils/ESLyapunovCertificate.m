classdef ESLyapunovCertificate < handle
%%ESLyapunovCertificate(V,a,k1,k2,k3 [,r] )
% 
% k1*||x||^a <=  V(t,x)   <= k2*||x||^a
%               dotV(t,x) <= -k3*||x||^a
%
% for all V(t,x) <= r (Default inf).
    
    properties
        a,k1,k2,k3,V,r=inf
    end
    
    methods
        
        function obj = ESLyapunovCertificate(V,a,k1,k2,k3,r)
            obj.a  = a;
            obj.k1 = k1;
            obj.k2 = k2;
            obj.k3 = k3;
            obj.V  = V;
            
            if nargin ==6
                obj.r = r;
            end
        end
        
    end
    
end

