
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
    % see also PolytopicSet, EllipsoidalSet, NonlinearSet
    
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