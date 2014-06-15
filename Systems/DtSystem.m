%
%DTSystem discrete time system
%
%   See help GeneralSystem
%
% DTSystem methods:
%
% DtSystem                     - costructor
% getStateTrajectory(obj,x0,u) - compute the state trajectory obtained
%                                from x0 applying the sequence of inputs u
%
%   See also GeneralSystem, CtSystem

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




classdef DtSystem < GeneralSystem
    
    
    properties
        
    end
    
    methods
        
        function obj = DtSystem (varargin)
            %A discrete time system can be created as
            %
            %   dtSys = DtSystem(par1,val1,par2,val2,...)
            %
            %   where the parameters and the associated values are specified in
            %   the help of the abstract class GeneralSystem, or as
            %   discretization of a continuous time system as
            %
            %   dtSys = DtSystem(ctSys,dt)
            %   dtSys = DtSystem(ctSys,dt,integrator)
            %
            %   where ctSys is of the class CtSystem, dt is the discretization
            %   step and integrator is an integration method (Default RK4)
            %
            %   See also GeneralSystem, CtSystem, Integrator, RK4
            
            
            if ( (nargin == 3 || nargin == 2) && isa(varargin{1},'CtSystem') )
                
                ctSys = varargin{1};
                
                superClassParameters = ctSys.getParameters();
                
                dt = varargin{2};
                
                indexF = find(strcmp(superClassParameters, 'StateEquation'));
                
                if isempty(ctSys.Q)
                    if nargin == 3
                        superClassParameters{indexF +1} = @(x,u) varargin{3}.integrate(@(y)ctSys.f(y,u),x,dt);
                    else
                        superClassParameters{indexF +1} = @(x,u) RK4.integrate(@(y)ctSys.f(y,u),x,dt);
                    end
                else
                    if nargin == 3
                        superClassParameters{indexF +1} = @(x,u,v) varargin{3}.integrate(@(y)ctSys.f(y,u,v),x,dt);
                    else
                        superClassParameters{indexF +1} = @(x,u,v) RK4.integrate(@(y)ctSys.f(y,u,v),x,dt);
                    end
                end
                %superClassParameters{indexF +1} = @(x,u) RK4.integrate(@(y)ctSys.f(y,u),x,dt);
                
                
            else
                
                superClassParameters = varargin;
                
            end
            
            obj = obj@GeneralSystem(superClassParameters{:});
            
        end
        
        
        function x = getStateTrajectory(obj,x0,u)
            
            x = zeros(length(x0),size(u,2)+1);
            x(:,1) = x0;
            
            for i =1:size(u,2)
                x(:,i+1) = obj.f(x(:,i),u(:,i));
            end
            
            if sum(sum(isnan(x)))>0 || sum(sum(isinf(x)))>0
                error('inf or nan prediction');
            end
            
        end
        
        function changeOfCoordinate(obj,varargin)
            % \bar{x} = Ax+b
            % \bar{u} = Cx+d
            
            A = eye(obj.nx);
            b = zeors(obj.nx,1);
            C = eye(obj.nu);
            d = zeors(obj.nu,1);
            
            if nargin >= 2 && not(isempty(varargin{1}))
                A = varargin{1};
            end
            if nargin >= 3 && not(isempty(varargin{2}))
                b = varargin{2};
            end
            
            if nargin >= 4 && not(isempty(varargin{3}))
                C = varargin{3};
            end
            
            if nargin >= 5 && not(isempty(varargin{4}))
                d = varargin{4};
            end
            
            
            if not(isempty(obj.f))
                fOld = obj.f;
                obj.f = @(x,u) A*fOld(A\(x-b),C\(u-d))+b;
                obj.cA = A;
                obj.cb = b;
            end
            
            if not(isempty(obj.h))
                hOld = obj.h;
                obj.h = @(x,u) hOld(A\(x-b),C\(u-d));
                obj.cC = C;
                obj.cd = d;
            end
            
            
            
        end
        
    end
end