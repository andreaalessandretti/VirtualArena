function [L,U] = findClosestIntersectionWithAPolytope(c,d,A,b)
%findClosestIntersectionWithAPolytope finds the upper and lower bounds on 
% alpha such that A(c+alpha*d)<=b
%
% if L>U there is not intersection
%
% e.g.
% S = BoxSet(-[2;2],1:2,-[1;1],1:2,2); 
% 
% d = [1;1];
% [L,U] = findClosestIntersectionWithAPolytope([0;0],d,S.A,S.b);
% if L>U
%     error('Unfeasible');
% end
% 
% if norm(L)<norm(U)
%     alpha = L;
% else
%     alpha = U;
% end
% 
% plot(S);
% hold on 
% plot([0,d(1)],[0,d(2)],'r');
% plot([0,alpha*d(1)],[0,alpha*d(2)],'r--');


 
% This file is part of VirtualArena.
%
% Copyright (c) 2014, Andrea Alessandretti
% All rights reserved.
%
% e-mail: andrea.alessandretti [at] {epfl.ch, ist.utl.pt}
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% 
% 1. Redistributions of source code must retain the above copyright notice, this
%    list of conditions and the following disclaimer. 
% 2. Redistributions in binary form must reproduce the above copyright notice,
%    this list of conditions and the following disclaimer in the documentation
%    and/or other materials provided with the distribution.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
% ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
% 
% The views and conclusions contained in the software and documentation are those
% of the authors and should not be interpreted as representing official policies, 
% either expressed or implied, of the FreeBSD Project.
 
    Ad = A*d;
    bmAc = b-A*c;
    
    
    r = bmAc./Ad;
    
    L = max(r(r==Inf | Ad<0));
    U = min(r(r==-Inf | Ad>0));
    
    
end

