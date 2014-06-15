function samples = generateSamples( varargin )
%generateSamples
%
% P = diag([2,1]);
% ellipplot(P);hold on
% aa = generateSamples('EllipticUniRand',P,P);
% plot(aa(1,:),aa(2,:),'o')
%
% ss = generateSamples('SquareCanvas',1,30);
% plot(ss(1,:),ss(2,:),'o')
%
% ss = generateSamples('EllipticCanvas',diag([2,1]),30);
% plot(ss(1,:),ss(2,:),'o')
%


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

switch varargin{1}
    case 'SquareCanvas'
        
        M = varargin{2};
        nn = varargin{3};
        samples = [linspace(-M,M,nn);
            M*ones(1,nn)];
        
        samples = [samples,...
            [linspace(-M,M,nn);
            -M*ones(1,nn)]];
        
        samples = [samples,...
            [M*ones(1,nn-2);
            linspace(-M+M/(nn-1),M-M/(nn-1),nn-2)]];
        
        
        samples = [samples,...
            [-M*ones(1,nn-2);
            linspace(-M+M/(nn-1),M-M/(nn-1),nn-2);]];
        if nargin >= 4
            offset = varargin{4};
            samples = samples + repmat(offset,1,nn*4-4);
        end
        
    case 'EllipticCanvas'
        P = varargin{2};
        nn = varargin{3};
        
        theta = linspace(0,2*pi,nn);
        scircle = [cos(theta);
            sin(theta)];
        samples= chol(P)\scircle;
        
    case 'EllipticUniRand'
        P = varargin{2};
        nx = size(P,1);
        nn = varargin{3};
        a = 1/sqrt(min(real(eigs(P))));
        samples = zeros(nx,nn);
        i=1;
        while i<=nn
            sample = 2*a*rand(nx,1)-a;
            if sample'*P*sample<=1
                samples(:,i) = sample;
                i=i+1;
            end
        end
        
        
    otherwise
        error('Generation type not recognised')
        
end
end
