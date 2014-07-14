
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
% Input vector:  u = [vk;wk;ud]
%
% - vk : actuation on the linear velocity of the vehicle
% - wk : actuation on the angular velocity of the vehicle
% - ud : actuation on the dynamicalm equation of the vehicle
%
% StateEquation :
%
% \dot{p} = R*(Lvk*vk + Lvd*vd + mv(x))
% \dot{R} = R*S(Lwk*wk + Lwd*wd + mw(x))
% \dot{d} = fd(x,uk) + Gd*ud
%
% Constructor :
%
% v = UnderactuatedVehicle('par1','val1',...)
%
% with paramteres:
% - contant matrices :'Lvk','Lvd','Lwk','Lwd','Gd'
% - function handles : 'fd','mv','md'
% 'AttitudeRepresentation' : 0 for quaternion representation,1 for StR
%               representation, i.e., R = [r1,r2,r3], StR = [r1;r2;r3]
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
        fd=[];
        Gd=[];
        mv = [];
        mw = [];
        attitudeRepresentation = 'RotationMatrix'; %0 = quaternion, 1 = rotation matrix
        getR
        getd
        getPosition
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
                            
                            obj.fd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                       case 'Gd'
                            
                            obj.Gd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                       
                       case 'mv'
                            
                            obj.mv = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                       case 'mw'
                            
                            obj.mw = varargin{parameterPointer+1};
                            
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
            
         
            
            %%

            
            
            %% compoe v
             if(isempty(obj.Lvk) && isempty(obj.Lvd))
                error('at least Lvk or Lvd must be defined')
             elseif(isempty(obj.Lvk) && not(isempty(obj.Lvd)))
                 obj.v = @(x,uk,d)obj.Lvd*d(1:size(obj.Lvd,2));
             elseif(not(isempty(obj.Lvk)) && isempty(obj.Lvd))
                 obj.v = @(x,uk,d)obj.Lvk*uk(1:obj.nvk);
             elseif(not(isempty(obj.Lvk)) && not(isempty(obj.Lvd)))
                 obj.v = @(x,uk,d)obj.Lvk*uk(1:obj.nvk)+obj.Lvd*d(1:size(obj.Lvd,2));
             end
             
                         
             if not(isempty(obj.mv))
                 obj.v = @(x,uk,d) obj.v(x,uk,d) +obj.mv(x);
             end
             
             %% compose omega
             if(isempty(obj.Lwk) && isempty(obj.Lwd))
                error('at least Lwk or Lwd must be defined')
             elseif(isempty(obj.Lwk) && not(isempty(obj.Lwd)))
                 obj.omega = @(x,uk,d)obj.Lwd*d(obj.nvd+1:obj.nvd+obj.nwd);
             elseif(not(isempty(obj.Lwk)) && isempty(obj.Lwd))
                 obj.omega = @(x,uk,d)obj.Lwk*uk(obj.nvk+1:obj.nvk+obj.nwk);
             elseif(not(isempty(obj.Lwk)) && not(isempty(obj.Lwd)))
                 obj.omega = @(x,uk,d)obj.Lwk*uk(obj.nvk+1:obj.nvk+obj.nwk)+obj.Lwd*d(obj.nvd+1:obj.nvd+obj.nwd);
             end
                        
             if not(isempty(obj.mw))
                 obj.omega = @(x,uk,d) obj.omega(x,uk,d) +obj.mw(x);
             end
             
            if(obj.n ==2)
                fk = @(x,u) UnderactuatedVehicle.fk2D(x(3),obj.v(x,u,obj.getd(x)),obj.omega(x,u,obj.getd(x)));
                obj.getR = @(x)[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
                obj.getPosition = @(x)x(1:2);
                
                nxk = 3; %dimension kinematic component of the state vector
            elseif(obj.n==3)
                obj.getPosition = @(x)x(1:3);
                switch obj.attitudeRepresentation 
                
                    case 'Quaternion' % Quaternions
                        fk = @(x,u) UnderactuatedVehicle.fk3DQuaternion(x(4:7),obj.v(x,u,obj.getd(x)),obj.omega(x,u,obj.getd(x)));
                        nxk = 7; %dimension kinematic component of the state vector
                        obj.getR = @(x)quaternion.q2R(x(4:7));
                    case 'RotationMatrix' % Rotation matrix
                        fk = @(x,u) UnderactuatedVehicle.fk3DRotMat(x(4:12),obj.v(x,u,obj.getd(x)),obj.omega(x,u,obj.getd(x)));
                        nxk = 12; %dimension kinematic component of the state vector
                        obj.getR = @(x)reshape(x(4:12),3,3);
                end 
            end
            
            offsetVW = obj.nwk +obj.nvk;
            if obj.nwd + obj.nvd >0
                ff = @(x,u)[fk(x,u);...
                               obj.fd(x,u(1:offsetVW))];
                           
                if not(isempty(obj.Gd))
                    ff = @(x,u)ff(x,u)+[zeros(nxk,1);obj.Gd*u(offsetVW+1:offsetVW+size(obj.Gd,2))];
                end
            else
                ff = fk;
            end
            
            obj.f = ff;
            
            obj.nx = nxk + obj.nwd +obj.nvd;
            obj.nu = obj.nwk + obj.nvk + size(obj.Gd,2);
            obj.getd = @(x)x(nxk+1:nxk+ obj.nwd +obj.nvd);
            
            
           
    end
    end
    
    methods (Static)
        
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