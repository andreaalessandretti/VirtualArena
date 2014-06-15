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
% Copyright (C) 2012-14 Andrea Alessandretti
%
% andrea.alessandretti@{ist.utl.pt, epfl.ch}
% Automatic Control Laboratory, EPFL, Lausanne, Switzerland.
% Institute System and Robotics, IST, Lisbon, Portugal.
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
    Ad = A*d;
    bmAc = b-A*c;
    
    
    r = bmAc./Ad;
    
    L = max(r(r==Inf | Ad<0));
    U = min(r(r==-Inf | Ad>0));
    
    
end

