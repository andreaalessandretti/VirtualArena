
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