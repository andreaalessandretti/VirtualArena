
classdef CtSystem < GeneralSystem
    %CtSystem Continuous Time System
    %
    %   See help GeneralSystem
    %
    %   CtSystem methods:
    %   CtSystem          - costructor sys = CtSystem(par1,val1,par2,val2, ...)
    %                       see GeneralSystem for the explanation of the
    %                       parameters and values.
    %   getStateTrajectory - Compute a solution
    %   changeOfCoordinate - Perform a change of state and/or input coordinate
    %
    % See also GeneralSystem, DtSystem
    
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
    
    properties (Access = private)
        
        % changeOfCoordinate - Variable of the change of choordinates
        % x' = Ax+b
        % u' = Cx+d
        
        cA = [];
        cb = [];
        cC = [];
        cd = [];
    end
    
    methods
        
        
        function obj = CtSystem (varargin)
            
            obj = obj@GeneralSystem(varargin{:});
            
        end
        
        
        
        function x = getStateTrajectory(obj,x0,u,dt)
            %getStateTrajectory Compute a solution
            %
            % Returns the state trajectory with the form [x0,x1,...]
            % obtained by solving recursively
            %
            % x(:,i+1) = x(:,i) + dt*sys.f(x(:,i),u(:,i));
            %
            % Calling the function:
            %
            % sys.getStateTrajectory(x0,u,dt)
            
            x = zeros(length(x0),size(u,2)+1);
            x(:,1) = x0;
            
            for i =1:size(u,2)
                x(:,i+1) = x(:,i) + dt*obj.f(x(:,i),u(:,i));
            end
            
            if sum(sum(isnan(x)))>0 || sum(sum(isinf(x)))>0
                error('The solution contains inf or nan elements');
            end
            
        end
        
        
        function changeOfCoordinate(obj,varargin)
            %changeOfCoordinate Perform a change of state and/or input coordinate
            %
            % The new system are will be expressed in the new state/input coordinate frame
            % x' = Ax+b
            % u' = Cu+d
            %
            % Calling the function:
            %
            % sys.changeOfCoordinate(A,b,C,d)
            %
            % Example:
            % -----------------------------------------------------------------
            % sys = CtSystem(...
            %     'StateEquation' ,@(x,u) x+u,'nx',1,'nu',1,...
            %     'OutputEquation',@(x,u) x-u,'ny',1);
            %
            % xDot = sys.f(3,7) % 10
            % y    = sys.h(3,7) % -4
            %
            % % x' = x+2, u' = 2*u+4
            %
            % sys.changeOfCoordinate(1,2,2,4);
            %
            %
            % % \dot{x'} = x+u = x'-2 + (1/2)u'-2
            % % y        = x-u = x'-2 - (1/2)u'+2
            %
            % xDot = sys.f(3,7) % 3-2 + (1/2)7 -2 = 2.5
            % y    = sys.h(3,7) % 3-2 - (1/2)7 +2 = -0.5
            % -----------------------------------------------------------------
            
            A = eye(obj.nx);
            b = zeros(obj.nx,1);
            C = eye(obj.nu);
            d = zeros(obj.nu,1);
            
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
                obj.f = @(x,u) A*fOld(A\(x-b),C\(u-d));
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