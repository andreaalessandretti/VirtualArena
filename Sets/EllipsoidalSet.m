

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