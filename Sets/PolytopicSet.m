
classdef PolytopicSet < GeneralSet
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
    %   See also BoxSet, GeneralSet, EllipsoidalSet
    
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
        A
        b
    end
    
    methods
        
        
        function obj = PolytopicSet(A,b)
            
            obj = obj@GeneralSet();
            
            obj.A = A;
            
            obj.b = b;
            
            obj.f = @(x) obj.A*x-obj.b ;
            
            obj.nx = size(obj.A,2);
            
            obj.nf = size(obj.A,1);
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
                
                error(getMessage('PolytopicSet:getAffineTransformation:DimensionsMismatch'))
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