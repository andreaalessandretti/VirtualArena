
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
    % Input vector:  u =�[vk;wk;ud]
    %
    % - vk : actuation on the linear velocity of the vehicle
    % - wk : actuation on the angular velocity of the vehicle
    % - ud : actuation on the dynamicalm equation of the vehicle
    %
    % StateEquation :
    %
    % \dot{p} = R*(Lvk*vk + Lvd*vd + vm(t,x))
    % \dot{R} = R*S(Lwk*wk + Lwd*wd + wm(t,x))
    % \dot{d} = fd(x,uk) + Gd*ud
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
    %
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
        Lvk=[];
        Lvd=[];
        Lwk=[];
        Lwd=[];
        nvd=[];
        nvk=[];
        nwk=[];
        nwd=[];
        nxk=[];
        
        fdh=[]; %h
        vmh = []; %h
        wmh = []; %h
        vmDoth = []; %h
        wmDoth = []; %h
        
        Gd=[];
        attitudeRepresentation = 'RotationMatrix'; %0 = quaternion, 1 = rotation matrix
        v
        omega
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
            
            if isempty(obj.Lvk)
                obj.nvk = 0;
            else
                obj.nvk = size(obj.Lvk,2);
            end
            
            if isempty(obj.Lvd)
                obj.nvd = 0;
            else
                obj.nvd = size(obj.Lvd,2);
            end
            
            if isempty(obj.Lwk)
                obj.nwk = 0;
            else
                obj.nwk = size(obj.Lwk,2);
            end
            
            if isempty(obj.Lwd)
                obj.nwd = 0;
            else
                obj.nwd = size(obj.Lwd,2);
            end
            
            
            %% compoe v
            if(isempty(obj.Lvk) && isempty(obj.Lvd))
                error('at least Lvk or Lvd must be defined')
            elseif(isempty(obj.Lvk) && not(isempty(obj.Lvd)))
                obj.v = @(t,x,uk,d)obj.Lvd*d(1:size(obj.Lvd,2))+obj.vm(t,x);
            elseif(not(isempty(obj.Lvk)) && isempty(obj.Lvd))
                obj.v = @(t,x,uk,d)obj.Lvk*uk(1:obj.nvk)+obj.vm(t,x);
            elseif(not(isempty(obj.Lvk)) && not(isempty(obj.Lvd)))
                obj.v = @(t,x,uk,d)obj.Lvk*uk(1:obj.nvk)+obj.Lvd*d(1:size(obj.Lvd,2))+obj.vm(t,x);
            end
            
            
            %% compose omega
            if(isempty(obj.Lwk) && isempty(obj.Lwd))
                error('at least Lwk or Lwd must be defined')
            elseif(isempty(obj.Lwk) && not(isempty(obj.Lwd)))
                obj.omega = @(t,x,uk,d)obj.Lwd*d(obj.nvd+1:obj.nvd+obj.nwd)+obj.wm(t,x);
            elseif(not(isempty(obj.Lwk)) && isempty(obj.Lwd))
                obj.omega = @(t,x,uk,d)obj.Lwk*uk(obj.nvk+1:obj.nvk+obj.nwk)+obj.wm(t,x);
            elseif(not(isempty(obj.Lwk)) && not(isempty(obj.Lwd)))
                obj.omega = @(t,x,uk,d)obj.Lwk*uk(obj.nvk+1:obj.nvk+obj.nwk)+obj.Lwd*d(obj.nvd+1:obj.nvd+obj.nwd)+obj.wm(t,x);
            end
            
            
            if(obj.n ==2)
                
                fk = @(t,x,u) UnderactuatedVehicle.fk2D(x(3),obj.v(t,x,u,obj.getd(x)),obj.omega(t,x,u,obj.getd(x)));
               
                nxk = 3; %dimension kinematic component of the state vector
                
            elseif(obj.n==3)
                switch obj.attitudeRepresentation
                    
                    case 'Quaternion' % Quaternions
                        fk = @(t,x,u) UnderactuatedVehicle.fk3DQuaternion(x(4:7),obj.v(t,x,u,obj.getd(x)),obj.omega(t,x,u,obj.getd(x)));
                        nxk = 7; %dimension kinematic component of the state vector
                    case 'RotationMatrix' % Rotation matrix
                        fk = @(t,x,u) UnderactuatedVehicle.fk3DRotMat(x(4:12),obj.v(t,x,u,obj.getd(x)),obj.omega(t,x,u,obj.getd(x)));
                        nxk = 12; %dimension kinematic component of the state vector
                    otherwise
                        error('AttitudeRepresentation not supported')
                end
            end
            
            offsetVW = obj.nwk + obj.nvk;
            if obj.nwd + obj.nvd >0
                ff = @(t,x,u)[fk(t,x,u);...
                              obj.fd(t,x,u(1:offsetVW))];
                
                if not(isempty(obj.Gd))
                    ff = @(t,x,u)ff(t,x,u)+[zeros(nxk,1);obj.Gd*u(offsetVW+1:offsetVW+size(obj.Gd,2))];
                end
            else
                ff = @(t,x,u)fk(t,x,u);
            end
            
            obj.f = @(varargin)UnderactuatedVehicle.dotX(ff,varargin);
            
            %% InputTransformation
            ff = obj.f;
            obj.f = @(t,x,u)ff(t,x,obj.Phi(t,x,u));
            
            obj.nx   = nxk + size(obj.Gd,1);
            obj.nu   = obj.nwk + obj.nvk + size(obj.Gd,2);
            obj.nxk = nxk;
            
            
            
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
                
                hP=patch(XY(:,1)+p(1),XY(:,2)+p(2),1);
                
            end
            
        end
        
        function R = getR(obj,x)
           if(obj.n ==2)
                
                R = [cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
                
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
        
        function d = getd(obj,x)
            d = x(obj.nxk+(1:size(obj.Gd,1)));
        end
        
        function uRealK = PhiK(obj,t,x,uAffineK)
            uRealK = uAffineK;
        end
        
        function uAffineK = PhiInvK(obj,t,x,uRealK)
            uAffineK = uRealK;
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
        
        function StPosTheta_dot = fk2D(theta,v,omega) % Theta is only for 3D case
            
            R = [cos(theta),-sin(theta);sin(theta),cos(theta)];
            
            dotp   = R*v;
            
            StPosTheta_dot = [dotp;omega];
        end
        
    end
    
    
    
    
end