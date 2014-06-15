% e.g.
%f = @(x,u)x^2*u+x-u;
%xbar =1;ubar =1;
%A = jacobianSamples(@(x)f(x,ubar),xbar)
%B = jacobianSamples(@(u)f(xbar,u),ubar)

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

function D = jacobianSamples(f,xbar,varargin)

if size(f(zeros(size(xbar))),1)==1
    f=@(x)f(x)';
end

if nargin > 2
    a = varargin{1};
else
    a = 0.0001;
end

nx = length(xbar);

dx = a*eye(nx);
invdxdxt = (1/a^2)*eye(nx); % inv(dx*dx')
celldx = mat2cell(dx,nx,ones(1,nx));
dfc = arrayfun(@(dx)( f(xbar+dx{:})-f(xbar) ),celldx,'UniformOutput',0);
df = cell2mat(dfc);
D = (df*dx')*invdxdxt';

end

% function D = jacobianSamples(f,xbar,varargin)
%  if nargin >2
%      a = varargin{1};
%  else
%      a = 0.1;
%  end
%     nx = length(xbar);
%     dx = a*eye(nx);
%     invdxdxt = (1/a^2)*eye(nx); % inv(dx*dx')
%     celldx = mat2cell(dx,nx,ones(1,nx));
%     dfc = arrayfun(@(dx)f(xbar+dx{:})'-f(xbar)',celldx,'UniformOutput',0);
%     df = cell2mat(dfc);
%
%
%     D = zeros(1,nx);
%
%    for j = 1:1000
%      for i=1:nx
%          y = df(:,i);
%          s = dx(:,i);
%
%          D = D + dx(:,i)'*df(:,i)/norm(dx(:,i))^2;
%
%      end
%    end
%     D = (df*dx')*invdxdxt';
%
% end
