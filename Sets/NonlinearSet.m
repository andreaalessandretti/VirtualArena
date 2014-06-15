
classdef NonlinearSet < handle
    %NonlinearSet
    %
    % The object
    %
    %   s = NonlinearSet(f,nx,nf)
    %
    % denotes the set
    %
    %   s =  {x:f_i(x) <= 0, i=1,...,nf, x \in R^nx}
    %
    %   See also BoxSet, PolytopicSet, EllipsoidalSet
    
    
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
        
        f
        
        nx
        
        nf
        
    end
    
    methods
        
        
        function obj = NonlinearSet(f,nx,nf)
            
            obj.f = f;
            
            obj.nx = nx;
            
            obj.nf = nf;
            
        end
        
        function params = getParameters(obj)
            
            params = {obj.f,obj.nx,obj.nf};
            
        end
        
        % Let AA be the ser {x:f(x)<0} and BB be defined as follows
        %
        % BB = AA.getAffinedTransformedSet(A,b)
        %
        % then BB = {x:f(Ax+b)<0}
        %
        function set = getAffineTransformation(obj,A,b)
            
            if ( not(obj.nx == size(A,1)) ||...
                    not(size(A,1) == size(b,1)) )
                error(getMessage('NonlinearSet:getAffineTransformation:DimentionsMismatch'))
            end
            
            ff = obj.f;
            set = NonlinearSet(@(x) ff(A*x+b),size(A,2));
        end
        
    end
    
end