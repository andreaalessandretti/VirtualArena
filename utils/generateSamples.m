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
