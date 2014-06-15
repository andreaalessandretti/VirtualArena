
classdef PolytopicSet < NonlinearSet
    %PolytopicSet
    %
    % The object
    %
    %   s = PolytopicSet(A,b)
    %
    % denotes the set
    %
    %   s =  {x:Ax  <= b}
    %
    %   See also BoxSet, NonlinearSet, EllipsoidalSet
    
    
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
        A
        b
    end
    
    methods
        
        
        function obj = PolytopicSet(A,b)
            
            obj = obj@NonlinearSet(@(x) A*x-b ,size(A,2),size(A,1));
            
            obj.A = A;
            
            obj.b = b;
            
        end
        
        % Let AA be the set {x:f(x)<0}
        %
        % AA.getAffineTransformation(C,c)
        %
        % AA = {x:f(Cx+c)<0}
        %
        function set = getAffineTransformation(obj,C,c)
            
            if ( not(obj.spaceDimension == size(C,1)) ||...
                    not(size(C,1) == size(c,1)) )
                
                error(getMessage('PolytopicSet:getAffineTransformation:DimentionsMismatch'))
            end
            
            set = PolytopicSet(obj.A*C,obj.b-obj.A*c);
            
        end
        
        
        function ret = plus(arg1,arg2)
            
            if not((isnumeric(arg1)||isnumeric(arg2)) && (isa(arg1,'PolytopicSet')||isa(arg2,'PolytopicSet')))
                error ('The sum is defined for a pair numeric and PolytopicSet');
            end
            
            if isnumeric(arg1)
                v = arg1;
                set = arg2;
            else
                v = arg2;
                set = arg1;
            end
            A = set.A;
            b = set.b;
            ret = PolytopicSet(A,b+A*v);
        end
        
        
        function ret = mtimes(arg1,arg2)
            
            if not((isnumeric(arg1)||isnumeric(arg2)) && (isa(arg1,'PolytopicSet')||isa(arg2,'PolytopicSet')))
                error ('The sum is defined for a pair numeric and PolytopicSet');
            end
            
            if isnumeric(arg1)
                P = arg1;
                set = arg2;
            else
                P = arg2;
                set = arg1;
            end
            A = set.A;
            b = set.b;
            ret = PolytopicSet(A/P,b);
        end
        
        function ret = vertcat(a,b)
            ret = PolytopicSet(blkdiag(a.A,b.A),[a.b;b.b]);
        end
        
    end
    
    
end