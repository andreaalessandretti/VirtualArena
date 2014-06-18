
classdef  BoxSet < PolytopicSet
    %BoxSet
    %
    % Constructor mothod:
    %
    % The object B
    %
    % B = BoxSet(lowerBounds,indexesLowerBounds,upperBounds,indexesUpperBounds,spaceDimension)
    %
    % denotes the box set
    %
    % B =  {x:   xi <= upperBounds(i) for all i in indexesUpperBounds
    %            xi >= lowerBounds(i) for all i in indexesLowerBounds  }
    %
    % with x \in R^spaceDimension.
    %
    %
    % BoxSet methods:
    %
    % project( v )
    %
    % b = BoxSet([-1;-1],1:2,[1;1],1:2,2) % {|x|_\infty <= 1 }
    % projectedV = b.project([3;0.5])     %=> [1;0.5]
    %
    % Operators:
    %
    % - vertcat [BoxSet;BoxSet]
    %
    % b1 = BoxSet(-1,1,1,1,1) % |x|<1
    % b2 = BoxSet(-2,1,2,1,1) % |x|<2
    % b = [b1;b2]             % {x : |x1|<1, |x2|<2}
    %
    % - add v + BoxSet
    %
    % b = BoxSet([-1;-1],1:2,[1;1],1:2,2) %{|x|_\infty <= 1 }
    % b = [1;1] + b                       %{|x-[1;1]|_\infty <= 1 }
    %
    % see also PolytopicSet, EllipsoidalSet, GeneralSet
    
 
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
        upperBounds
        indexesUpperBounds
        lowerBounds
        indexesLowerBounds
        spaceDimension
    end
    
    methods
        
        
        
        function obj = BoxSet(lowerBounds,indexesLowerBounds,upperBounds,indexesUpperBounds,spaceDimension)
            
            if not((length(indexesLowerBounds) == length(lowerBounds)) &&(length(indexesUpperBounds) == length(upperBounds)))
                error('The dimention of the indexes vector does not match with the bound vector ');
            end
            
            I = eye(spaceDimension);
            
            A = [I(indexesUpperBounds,:);
                -I(indexesLowerBounds,:)];
            
            b =  [upperBounds;
                -lowerBounds];
            
            obj = obj@PolytopicSet(A,b);
            
            obj.upperBounds = upperBounds;
            
            obj.indexesUpperBounds = indexesUpperBounds;
            
            obj.lowerBounds = lowerBounds;
            
            obj.indexesLowerBounds = indexesLowerBounds;
            
            obj.spaceDimension = spaceDimension;
            
        end
        
        
        function vProjected = project(obj,v)
            %% set.project(v) projects the vector v in the set set.
            %
            % e.g.
            %
            % set = BoxSet([-1;-2;4],1:3,[3;3;5],1:3,3);
            % vProjected = set.project([-2;2;6])
            %
            % vPeojected = [-1;2;5]
            
            vProjected = v;
            
            boundedV = vProjected(obj.indexesUpperBounds);
            idxViolation = (boundedV>obj.upperBounds);
            boundedV(idxViolation) =obj.upperBounds(idxViolation);
            vProjected(obj.indexesUpperBounds) = boundedV;
            
            boundedV = vProjected(obj.indexesLowerBounds);
            idxViolation = (boundedV<obj.lowerBounds);
            boundedV(idxViolation) =obj.lowerBounds(idxViolation);
            vProjected(obj.indexesLowerBounds) = boundedV;
            
            
            
            
        end
        
        function plot(obj,varargin)
            plot([obj.lowerBounds(1),obj.lowerBounds(1),obj.upperBounds(1),obj.upperBounds(1),obj.lowerBounds(1)],...
                [obj.lowerBounds(2),obj.upperBounds(2),obj.upperBounds(2),obj.lowerBounds(2),obj.lowerBounds(2)],varargin{:});
        end
        
        %Note: the addition is performed only in the existing bounds
        function ret = plus(arg1,arg2)
            
            if not((isnumeric(arg1)||isnumeric(arg2)) && (isa(arg1,'PolytopicSet')||isa(arg2,'PolytopicSet')))
                error ('The sum is defined for a pair numeric and BoxSet');
            end
            
            
            
            if isnumeric(arg1)
                v = arg1;
                set = arg2;
            else
                v = arg2;
                set = arg1;
            end
            
            oldUpperBounds = set.upperBounds;
            oldLowerBounds = set.lowerBounds;
            oldIndexesUpperBounds = set.indexesUpperBounds;
            oldIndexesLowerBounds = set.indexesLowerBounds;
            oldSpaceDimension = set.spaceDimension;
            
            if not(length(v)==oldSpaceDimension)
                error ('The vector has to be of the same dimention of the set');
            end
            
            
            ret = BoxSet(oldLowerBounds+v(oldIndexesLowerBounds),oldIndexesLowerBounds,...
                oldUpperBounds+v(oldIndexesUpperBounds),oldIndexesUpperBounds,...
                oldSpaceDimension);
        end
        
        function ret = mtimes(arg1,arg2)
            
            if not((isnumeric(arg1)||isnumeric(arg2)) && (isa(arg1,'PolytopicSet')||isa(arg2,'PolytopicSet')))
                error ('The sum is defined for a pair numeric and BoxSet');
            end
            
            if isnumeric(arg1)
                P = arg1;
                set = arg2;
            else
                P = arg2;
                set = arg1;
            end
            
            oldUpperBounds = set.upperBounds;
            oldLowerBounds = set.lowerBounds;
            oldIndexesUpperBounds = set.indexesUpperBounds;
            oldIndexesLowerBounds = set.indexesLowerBounds;
            oldSpaceDimension = set.spaceDimension;
            
            
            PIsDiagonal = norm(P-diag(diag(P)))==0;
            
            if PIsDiagonal
                
                diagP =diag(P);
                
                if not(length(diagP)==oldSpaceDimension)
                    error ('The matrix has to be of the same dimention of the set');
                end
                
                
                newIndexesLowerBounds = oldIndexesLowerBounds;
                newIndexesUpperBounds = oldIndexesUpperBounds;
                
                newLowerBounds = oldLowerBounds.*diagP(newIndexesLowerBounds);
                newUpperBounds = oldUpperBounds.*diagP(newIndexesUpperBounds);
                
                for i=1:length(diagP)
                    
                    if (diagP(i)<00)
                        isLb = find(newIndexesLowerBounds==i);
                        isUb = find(newIndexesUpperBounds==i);
                        
                        if isLb & isUb %Switch
                            
                            olb = newLowerBounds(isLb);
                            oub = newUpperBounds(isUb);
                            
                            newLowerBounds(isLb)=oub;
                            newUpperBounds(isUb)=olb;
                            
                        elseif isLb
                            
                            newUpperBounds = [newUpperBounds;newLowerBounds(isLb)];
                            newIndexesUpperBounds = [newIndexesUpperBounds,newIndexesLowerBounds(isLb)];
                            
                            
                            %newLowerBounds = [newLowerBounds(1:max(isLb-1,0));newLowerBounds(isLb+1:end)];
                            %newIndexesLowerBounds = [newIndexesLowerBounds(1:max(isLb-1,0)),newIndexesLowerBounds(isLb+1:end)];
                            
                            if isempty(newLowerBounds(1:max(isLb-1,0))) & isempty(newLowerBounds(isLb+1:end))
                                newLowerBounds = [];
                            elseif isempty(newLowerBounds(1:max(isLb-1,0)))
                                newLowerBounds = newLowerBounds(isLb+1:end);
                            elseif newLowerBounds(isLb+1:end)
                                newLowerBounds = newLowerBounds(1:max(isLb-1,0));
                            else
                                newLowerBounds = [newLowerBounds(1:max(isLb-1,0));newLowerBounds(isLb+1:end)];
                            end
                            
                            if isempty(newIndexesLowerBounds(1:max(isLb-1,0))) & isempty(newIndexesLowerBounds(isLb+1:end))
                                newIndexesLowerBounds = [];
                            elseif isempty(newIndexesLowerBounds(1:max(isLb-1,0)))
                                newIndexesLowerBounds = newIndexesLowerBounds(isLb+1:end);
                            elseif isempty(newIndexesLowerBounds(isLb+1:end))
                                newIndexesLowerBounds = newIndexesLowerBounds(1:max(isLb-1,0));
                            else
                                newIndexesLowerBounds = [newIndexesLowerBounds(1:max(isLb-1,0)),newIndexesLowerBounds(isLb+1:end)];
                            end
                            
                        elseif isUb
                            
                            newLowerBounds = [newLowerBounds;newUpperBounds(isUb)];
                            newIndexesLowerBounds = [newIndexesLowerBounds,newIndexesUpperBounds(isUb)];
                            
                            %newUpperBounds = [newUpperBounds(1:max(isUb-1,0));newUpperBounds(isUb+1:end)];
                            %newIndexesUpperBounds = [newIndexesUpperBounds(1:max(isUb-1,0)),newIndexesUpperBounds(isUb+1:end)];
                            
                            if isempty(newUpperBounds(1:max(isUb-1,0))) & isempty (newUpperBounds(isUb+1:end))
                                newUpperBounds = [];
                            elseif isempty(newUpperBounds(1:max(isUb-1,0)))
                                newUpperBounds = newUpperBounds(isUb+1:end);
                            elseif newUpperBounds(isUb+1:end)
                                newUpperBounds = newUpperBounds(1:max(isUb-1,0));
                            else
                                newUpperBounds = [newUpperBounds(1:max(isUb-1,0));newUpperBounds(isUb+1:end)];
                            end
                            
                            if isempty(newIndexesUpperBounds(1:max(isUb-1,0))) & isempty(newIndexesUpperBounds(isUb+1:end))
                                newIndexesUpperBounds = [];
                            elseif isempty(newIndexesUpperBounds(1:max(isUb-1,0)))
                                newIndexesUpperBounds = newIndexesUpperBounds(isUb+1:end);
                            elseif isempty(newIndexesUpperBounds(isUb+1:end))
                                newIndexesUpperBounds = newIndexesUpperBounds(1:max(isUb-1,0));
                            else
                                newIndexesUpperBounds = [newIndexesUpperBounds(1:max(isUb-1,0)),newIndexesUpperBounds(isUb+1:end)];
                            end
                            
                        end
                        
                    end
                end
                
                ret = BoxSet(newLowerBounds,newIndexesLowerBounds,...
                    newUpperBounds,newIndexesUpperBounds,...
                    oldSpaceDimension);
                
            else
                ret = mtimes@PolytopicSet(arg1,arg2);
                
            end
            
        end
        
        
        function ret = vertcat(a,b)
            
            ret = BoxSet(...
                [a.lowerBounds;b.lowerBounds],...
                [a.indexesLowerBounds,b.indexesLowerBounds+a.spaceDimension*ones(size(b.indexesLowerBounds))],...
                [a.upperBounds;b.upperBounds],...
                [a.indexesUpperBounds,b.indexesUpperBounds+a.spaceDimension*ones(size(b.indexesUpperBounds))],...
                a.spaceDimension+b.spaceDimension...
                );
        end
        
        %        % I multiply the bounds by a scalar
        %         function ret = mtimes(a,b)
        %             if ( isnumeric(a) && isa(b,'BoxSet') )
        %                 set = b; n = a;
        %             elseif ( isnumeric(b) && isa(a,'BoxSet') )
        %                  set = a; n = b;
        %             else
        %
        %                 error('BoxSet moltiplication is defined between a number as a BoxSet object only.')
        %             end
        %                 set.upperBounds = n*set.upperBounds;
        %                 set.lowerBounds = n*set.lowerBounds;
        %                 ret = BoxSet(set.lowerBounds,set.indexesLowerBounds,set.upperBounds,set.indexesUpperBounds,set.spaceDimension);
        %
        %         end
        
        
    end
end