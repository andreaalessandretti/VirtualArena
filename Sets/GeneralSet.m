
classdef GeneralSet < handle
    %GeneralSet
    %
    % The object
    %
    %   s = GeneralSet(f,nx,nf)
    %
    % denotes the set
    %
    %   s =  {x:f_i(x) <= 0, i=1,...,nf, x \in R^nx}
    %
    %   See also BoxSet, PolytopicSet, EllipsoidalSet
    
    
    
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
        
        f
        
        nx
        
        nf
        
    end
    
    methods
        
        
        function obj = GeneralSet(f,nx,nf)
            if nargin > 0
                obj.f = f;

                obj.nx = nx;

                obj.nf = nf;
            end
            
        end
        
        function params = getParameters(obj)
            
            params = {obj.f,obj.nx,obj.nf};
            
        end
        
        % Let AA be the ser {x:f(x)<0} and BB be defined as follows
        %
        % BB = AA.getAffinedTransformedSet(A,b)
        %
        % then BB = {x:f(Ax+b)<0}
        
        function set = getAffineTransformation(obj,A,b)
            
            if ( not(obj.nx == size(A,1)) ||...
                    not(size(A,1) == size(b,1)) )
                error(getMessage('GeneralSet:getAffineTransformation:DimensionsMismatch'))
            end
            
            ff = obj.f;
            set = GeneralSet(@(x) ff(A*x+b),size(A,2));
        end
        
        
        function ret = contains(obj,v)
            ret = sum(obj.f(v)<=0)==obj.nf;
        end
        
    end
    
end