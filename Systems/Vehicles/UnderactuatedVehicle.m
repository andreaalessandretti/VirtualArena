
classdef UnderactuatedVehicle < Vehicle
    %UnderactuatedVehicle
    %
    % State vector: x = [p;par{R};d]
    %
    % - p      : 2-D/2-D vector denoting the position of the vehicle
    % - par{R} : parametrization of the rotation matrix from body to inertia
    %            frame R, e.g, quaternion, columns of R, ...
    % - d      : dynamic component d := [vd;wd], see model below
    %
    % Input vector:  u =[vk;wk;ud]
    %
    % - vk : actuation on the linear velocity of the vehicle
    % - wk : actuation on the angular velocity of the vehicle
    % - ud : actuation on the dynamicalm equation of the vehicle
    %
    % StateEquation :
    %
    % \dot{p} = R*(Lvk*vk + Lvd*vd + vm(t,x))
    % \dot{R} = R*S(Lwk*wk + Lwd*wd + wm(t,x))
    % \dot{d} = fd(t,x,uk) + Gd*ud
    %
    % Constructor :
    %
    % v = UnderactuatedVehicle('par1','val1',...)
    %
    % with paramteres:
    % - contant matrices :'Lvk','Lvd','Lwk','Lwd','Gd'
    % - function handles : 'fd','vm','md'
    % 'AttitudeRepresentation' : 0 for quaternion representation,1 for StR
    %               representation, i.e., R = [r1,r2,r3], StR = [r1;r2;r3]
    %
    % 'uModel2uReal' function @(t,x,ud) -> uReal
    % 'PositionSpaceDimension'
    % Not all the parameters are needed, see e.g., Unicycle class
    %
    % See also Vehicle, Unicycle, UAV
    
    
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
        
        %% System Matrices
        % see help
        
        Lvk = [];
        Lvd = [];
        Lwk = [];
        Lwd = [];
        Gd  = [];
        
        %% Vector Dimentions
        
        nvd = [];
        nvk = [];
        nwd = [];
        nwk = [];
        nxk = []; %Dimension kinematic component of the state vector (position + heading representation)
        nd  = []; %Dimension dynamic component of the state vector (nvd + nwd for now)
        
        %Function Handles
        fdh    = [];
        fk     = [];
        vmh    = [];
        wmh    = [];
        vmDoth = [];
        wmDoth = [];
        
        v      = [];
        omega  = [];
        
        
        attitudeRepresentation = 'RotationMatrix'; %0 = quaternion, 1 = rotation matrix
        
        
    end
    
    methods
        
        function obj =  UnderactuatedVehicle(varargin)
            
            obj = obj@Vehicle(varargin{:});
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'Lvk'
                            
                            obj.Lvk = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Lvd'
                            
                            obj.Lvd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Lwk'
                            
                            obj.Lwk = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Lwd'
                            
                            obj.Lwd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'fd'
                            
                            obj.fdh = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Gd'
                            
                            obj.Gd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'vm'
                            
                            obj.vmh = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                        case 'wm'
                            
                            obj.wmh = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'AttitudeRepresentation'
                            
                            obj.attitudeRepresentation = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                    
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            obj.nvk = va_iff( isempty(obj.Lvk), 0, size(obj.Lvk,2));
            
            obj.nvd = va_iff( isempty(obj.Lvd), 0, size(obj.Lvd,2));
            
            obj.nwk = va_iff( isempty(obj.Lwk), 0, size(obj.Lwk,2));
            
            obj.nwd = va_iff( isempty(obj.Lwd), 0, size(obj.Lwd,2));
            
            
            %% Compose v
            if(isempty(obj.Lvk) && isempty(obj.Lvd))
                error('Either Lvk or Lvd must be defined.')
            elseif(isempty(obj.Lvk) && not(isempty(obj.Lvd)))
                obj.v = @(t,x,uk,d)obj.Lvd*d(1:obj.nvd)+obj.vm(t,x);
            elseif(not(isempty(obj.Lvk)) && isempty(obj.Lvd))
                obj.v = @(t,x,uk,d)obj.Lvk*uk(1:obj.nvk)+obj.vm(t,x);
            elseif(not(isempty(obj.Lvk)) && not(isempty(obj.Lvd)))
                obj.v = @(t,x,uk,d)obj.Lvk*uk(1:obj.nvk)+obj.Lvd*d(1:size(obj.Lvd,2))+obj.vm(t,x);
            end
            
            %% Compose omega
            if(isempty(obj.Lwk) && isempty(obj.Lwd))
                error('Either Lwk or Lwd must be defined.')
            elseif(isempty(obj.Lwk) && not(isempty(obj.Lwd)))
                obj.omega = @(t,x,uk,d)obj.Lwd*d(obj.nvd+1:obj.nvd+obj.nwd)+obj.wm(t,x);
            elseif(not(isempty(obj.Lwk)) && isempty(obj.Lwd))
                obj.omega = @(t,x,uk,d)obj.Lwk*uk(obj.nvk+1:obj.nvk+obj.nwk)+obj.wm(t,x);
            elseif(not(isempty(obj.Lwk)) && not(isempty(obj.Lwd)))
                obj.omega = @(t,x,uk,d)obj.Lwk*uk(obj.nvk+1:obj.nvk+obj.nwk)+obj.Lwd*d(obj.nvd+1:obj.nvd+obj.nwd)+obj.wm(t,x);
            end
            
            %% Build Kinematic Component of the state vector including attitude parametrization
            if(obj.n == 2)
                
                %fk  = @(t,x,u) UnderactuatedVehicle.fk2D(x(3),obj.v(t,x,u,obj.getd(x)),obj.omega(t,x,u,obj.getd(x)));
                nxk = 3;
                
            elseif(obj.n==3)
                
                switch obj.attitudeRepresentation
                    
                    case 'Quaternion' % Quaternions
                        
                        nxk = 7;
                        %fk  = @(t,x,u) UnderactuatedVehicle.fk3DQuaternion(x(4:nxk),obj.v(t,x,u,obj.getd(x)),obj.omega(t,x,u,obj.getd(x)));
                        
                    case 'RotationMatrix' % Rotation matrix
                        
                        nxk = 12;
                        %fk  = @(t,x,u) UnderactuatedVehicle.fk3DRotMat(x(4:nxk),obj.v(t,x,u,obj.getd(x)),obj.omega(t,x,u,obj.getd(x)));
                        
                    otherwise
                        
                        error('Given AttitudeRepresentation not supported.');
                        
                end
                
            end
            
%            obj.fk   = fk;
            
            nuk = obj.nwk + obj.nvk;
            nd  = obj.nwd + obj.nvd;
            
%             %% Compose f (merge kinematic and dynamic)
%             if nd >0
%                 
%                 ff = @(t,x,u)[fk(t,x,u); obj.fd(t,x,u(1:nuk))];
%                 
%                 if not(isempty(obj.Gd))
%                     ff = @(t,x,u)ff(t,x,u)+[zeros(nxk,1);obj.Gd*u(nuk+1:nuk+size(obj.Gd,2))];
%                 end
%                 
%             else
%                 
%                 ff = @(t,x,u)fk(t,x,u);
%                 
%             end
            
            %obj.f = @(varargin)UnderactuatedVehicle.dotX(ff,varargin);
            
            %% InputTransformation
            %ff      = obj.f;
            %obj.f   = @(t,x,u)ff(t,x,obj.Phi(t,x,u));
            obj.nd  = nd;
            obj.nxk = nxk;
            obj.nx  = obj.nxk + obj.nd;
            
            if isempty(obj.RealNu())
                obj.nu = obj.nwk + obj.nvk + size(obj.Gd,2);
            else
                obj.nu = obj.RealNu();
            end
            
            
        end
        
        function xDot =  f(obj,t,x,u,varargin)
            
            
            %% Build Kinematic Component of the state vector including attitude parametrization
            if(obj.n == 2)
                
                fk  = UnderactuatedVehicle.fk2D(x(3),obj.v(t,x,u,obj.getd(x)),obj.omega(t,x,u,obj.getd(x)));
                nxk = 3;
                
            elseif(obj.n==3)
                
                switch obj.attitudeRepresentation
                    
                    case 'Quaternion' % Quaternions
                        
                        nxk = 7;
                        fk  =  UnderactuatedVehicle.fk3DQuaternion(x(4:nxk),obj.v(t,x,u,obj.getd(x)),obj.omega(t,x,u,obj.getd(x)));
                        
                    case 'RotationMatrix' % Rotation matrix
                        
                        nxk = 12;
                        fk  = UnderactuatedVehicle.fk3DRotMat(x(4:nxk),obj.v(t,x,u,obj.getd(x)),obj.omega(t,x,u,obj.getd(x)));
                        
                    otherwise
                        
                        error('Given AttitudeRepresentation not supported.');
                        
                end
                
            end
            
            
            nuk = obj.nwk + obj.nvk;
            nd  = obj.nwd + obj.nvd;
            
            %% Compose f (merge kinematic and dynamic)
            if nd >0
                
                ff =[fk; obj.fd(t,x,u(1:nuk))];
                
                if not(isempty(obj.Gd))
                    ff = ff+[zeros(nxk,1);obj.Gd*u(nuk+1:nuk+size(obj.Gd,2))];
                end
                
            else
                
                ff = fk;
                
            end
            
            %f = UnderactuatedVehicle.dotX(ff,varargin);
            
            %% InputTransformation
            f = ff;
            %f   = ff(t,x,obj.Phi(t,x,u));
            %obj.nd  = nd;
            %obj.nxk = nxk;
            %obj.nx  = obj.nxk + obj.nd;
            
            %if isempty(obj.RealNu())
            %    obj.nu = obj.nwk + obj.nvk + size(obj.Gd,2);
            %else
            %    obj.nu = obj.RealNu();
            %end
            
            xDot = f;
        end
        
        
        function hP = plot(obj,varargin)
            
            if isempty(varargin)
                x  = obj.x;
            else
                x = varargin{1};
            end
            
            p = obj.getPosition(x);
            
            if length(p)==2
                
                R = obj.getR(x);
                
                h = 1;
                w = h/2;
                k = 0.5;
                
                XY=[ -h/2,-w/2;
                    -h/2,w/2  ;
                    h/2,k*w/2  ;
                    h/2,-k*w/2 ]*R';
                
                if nargin>2
                    hP=patch(XY(:,1)+p(1),XY(:,2)+p(2),1,varargin{2},varargin{3});
                else
                    hP=patch(XY(:,1)+p(1),XY(:,2)+p(2),1);
                end
                
                
            elseif length(p)==3
                
                R = obj.getR(x);
                
                l = 0.3*1;
                w = 0.3*1;
                h = 0.3*0.1;
                r = 0.3*0.1;
                
                X = [-l/2,    l/2, -l/2, -l/2, -l/2, -l/2;
                    -l/2,    l/2, -l/2, -l/2, -l/2, -l/2;
                    -l/2,    l/2,  l/2, l/2,  l/2, l/2;
                    -l/2,    l/2,  l/2, l/2,  l/2, l/2];
                
                Y =[ w/2,  r*w/2,  w/2,   w/2,   -w/2,  -w/2;
                    0,      0,    0,     0,      0,     0;
                    -w/2, -r*w/2,    0,     0,      0,     0;
                    0,      0,r*w/2, r*w/2, -r*w/2, -r*w/2 ];
                
                Z = [   0,      0,    0,    0,    0,    0;
                    h/2,  r*h/2,  h/2, -h/2,  h/2, -h/2;
                    0,      0, r*h/2, -r*h/2, r*h/2, -r*h/2;
                    -h/2, -r*h/2,     0,  0 ,     0,  0 ];
                
                xyz_points = [reshape(X,1,size(X,1)*size(X,2));
                    reshape(Y,1,size(Y,1)*size(Y,2));
                    reshape(Z,1,size(Z,1)*size(Z,2))];
                
                
                xyz_new = repmat(p,1,size(xyz_points,2)) +  R*xyz_points;
                
                Xnew = reshape(xyz_new(1,:),size(X,1),size(X,2));
                Ynew = reshape(xyz_new(2,:),size(Y,1),size(Y,2));
                Znew = reshape(xyz_new(3,:),size(Z,1),size(Z,2));
                
                h = fill3(Xnew,Ynew,Znew,0.5*ones(4,6));
            end
            
        end
        
        function set.Gd(obj,Gd)
            
            if obj.nd >0 && isempty(obj.Gd)
                
                nuk = obj.nwk + obj.nvk;
                
                obj.Gd = Gd;
                ff = @(t,x,u)[obj.fk(t,x,u)                ;...
                    obj.fd(t,x,u(1:nuk))]+[zeros(obj.nxk,1);obj.Gd*u(nuk+1:nuk+size(obj.Gd,2))];
                obj.f = @(varargin)UnderactuatedVehicle.dotX(ff,varargin);
            else
                obj.Gd = Gd;
            end
            
            if isempty(obj.RealNu())
                obj.nu = obj.nwk + obj.nvk + size(obj.Gd,2);
            end
            
        end
        
        
        function R = getR(obj,x)
            if(obj.n ==2)
                
                R = [cos(x(3)),-sin(x(3));
                    sin(x(3)),cos(x(3))];
                
            elseif(obj.n==3)
                
                switch obj.attitudeRepresentation
                    
                    case 'Quaternion' % Quaternions
                        R = quaternion.q2R(x(4:7));
                        
                    case 'RotationMatrix' % Rotation matrix
                        R = reshape(x(4:12),3,3);
                end
            end
        end
        
        
        function p = getPosition(obj,x)
            p = x(1:obj.n);
        end
        
        
        
        
        function d = getd(obj,x)
            d = x(obj.nxk+(1:obj.nd));
        end
        
        function uRealK = PhiK(obj,t,x,uAffineK)
            uRealK = uAffineK;
        end
        
        function uAffineK = PhiInvK(obj,t,x,uRealK)
            uAffineK = uRealK;
        end
        
        function realNu = RealNu(obj)
            realNu = [];
        end
        
        function uRealD = PhiD(obj,t,x,uAffineD)
            uRealD = uAffineD;
        end
        
        function uAffineD = PhiInvD(obj,t,x,uRealD)
            uAffineD = uRealD;
        end
        
        function uReal = PhiInv(obj,t,x,uAffine)
            nuk = obj.nvk+obj.nwk;
            nud = size(obj.Gd,2);
            if nuk>0 & nud>0
                uReal = [
                    PhiInvK(obj,t,x,uAffine(1:nuk));
                    PhiInvD(obj,t,x,uAffine(nuk+1:nuk+nud));
                    ];
            elseif nuk>0
                uReal = PhiInvK(obj,t,x,uAffine(1:nuk));
            elseif nud>0
                uReal = PhiInvD(obj,t,x,uAffine(nuk+1:nuk+nud));
            else
                error('nopee');
            end
        end
        
        function uAffine = Phi(obj,t,x,uReal)
            nuk = obj.nvk+obj.nwk;
            nud = size(obj.Gd,2);
            if nuk>0 & nud>0
                uAffine = [
                    PhiK(obj,t,x,uReal(1:nuk));
                    PhiD(obj,t,x,uReal(nuk+1:nuk+nud));
                    ];
            elseif nuk>0
                uAffine = PhiK(obj,t,x,uReal(1:nuk));
            elseif nud>0
                uAffine = PhiD(obj,t,x,uReal(nuk+1:nuk+nud));
            else
                error('nopee');
            end
            
        end
        
        %% The following function can be overwritten by subclasses
        function val = fd(obj,t,x,u)
            
            if ~isempty(obj.fdh);
                val = obj.fdh(t,x,u);
            end
            
        end
        
        function val = wmDot(obj,t,x)
            
            if isempty(obj.wmDoth);
                if obj.n == 2
                    val = 0;
                elseif obj.n == 3
                    val = zeros(3,1);
                end
            else
                val = obj.wmDoth(t,x);
            end
            
        end
        
        function val = vmDot(obj,t,x)
            
            if isempty(obj.vmDoth);
                val = zeros(obj.n,1);
            else
                val = obj.vmDoth(t,x);
            end
            
        end
        
        function val = wm(obj,t,x)
            
            if isempty(obj.wmh);
                if obj.n == 2
                    val = 0;
                elseif obj.n == 3
                    val = zeros(3,1);
                end
            else
                val = obj.wmh(t,x);
            end
            
        end
        
        function val = vm(obj,t,x)
            
            if isempty(obj.vmh);
                val = zeros(obj.n,1);
            else
                val = obj.vmh(t,x);
            end
            
        end
        
    end
    
    methods (Static)
        
        function dotX = dotX(ff,varargin)
            if length(varargin{:}) == 3
                dotX = ff(varargin{:}{:});
            elseif length(varargin{:}) == 4
                dotX = ff(varargin{:}{1:3})+varargin{:}{4};
            else
                error('?');
            end
            
        end
        
        function StPosQuat_dot = fk3DQuaternion(quat,v,omega) % quaternon is only for 3D case
            
            E1 = [1-2*(quat(2)^2+quat(3)^2)          , 2*(quat(1)*quat(2)-quat(3)*quat(4)), 2*(quat(1)*quat(3)+quat(2)*quat(4));
                2*(quat(1)*quat(2)+quat(3)*quat(4)), 1-2*(quat(1)^2+quat(3)^2)          , 2*(quat(2)*quat(3)-quat(1)*quat(4));
                2*(quat(1)*quat(3)-quat(2)*quat(4)), 2*(quat(2)*quat(3)+quat(1)*quat(4)), 1-2*(quat(1)^2+quat(2)^2)          ];
            
            E2 = 0.5.*[ quat(4), -quat(3),  quat(2);
                quat(3),  quat(4), -quat(1);
                -quat(2),  quat(1),  quat(4);
                -quat(1), -quat(2), -quat(3)];
            
            dot_p     = E1*v;
            dot_quat  = E2*omega;
            
            
            StPosQuat_dot = [dot_p;dot_quat];
        end
        
        function StPosStR_dot = fk3DRotMat(StR,v,omega) % RotMat is only for 3D case
            
            R = reshape(StR,3,3);
            
            S = [ 0       ,-omega(3) ,  omega(2);
                omega(3), 0        , -omega(1);
                -omega(2), omega(1) ,  0];
            
            dotp   = R*v;
            dotR   = R*S;
            stDotR = reshape(dotR,9,1);
            
            StPosStR_dot = [dotp;stDotR];
        end
        
        function StPosTheta_dot = fk2D(theta,v,omega) % Theta is only for 2D case
            
            R = [cos(theta),-sin(theta);
                sin(theta), cos(theta)];
            
            dotp = R*v;
            
            StPosTheta_dot = [dotp;omega];
        end
        
    end
    
end