

classdef EllipsoidalSet < NonlinearSet
    %EllipsoidalSet {x: (x-c)'*P*(x-c) <= b}
    %
    % The objects s1 and s2 defined as
    %
    %   s1 = EllipticSet(P,b);
    %   s2 = EllipticSet(P,b,c);
    %
    % denote the sets
    %
    %   s1 = {x: x'*P*x <= b}
    %   s2 = {x: (x-c)'*P*(x-c) <= b}
    %
    %   See also BoxSet
    
 
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
    
    properties
        P
        b
        c
    end
    
    methods
        
        
        function obj = EllipticSet(varargin)
            
            if nargin >= 2
                P = varargin{1};
                b = varargin{2};
                c = zeros(length(varargin{1}),1);
            end
            
            if nargin >= 3
                c = varargin{3};
            end
            
            obj = obj@NonlinearSet(@(x) (x-c)'*P*(x-c)-b ,size(P,2),1);
            
            obj.P = P;
            
            obj.b = b;
            
            obj.c = c;
            
        end
        
        
        
        
        
        function ret = mtimes(arg1,arg2)
            
            if not(isnumeric(arg1) && isa(arg2,'EllipticSet'))
                error ('The product is defined for a pair Matrix*EllipticSet');
            end
            
            
            A = arg1;
            set = arg2;
            
            invA = inv(A);
            
            ret = EllipticSet(invA'*set.P*invA,set.b);
        end
        
    end
end