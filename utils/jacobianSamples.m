% e.g.
%f = @(x,u)x^2*u+x-u;
%xbar =1;ubar =1;
%A = jacobianSamples(@(x)f(x,ubar),xbar)
%B = jacobianSamples(@(u)f(xbar,u),ubar)

 
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
