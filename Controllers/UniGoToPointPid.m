

classdef UniGoToPointPid < DtSystem
    %UniGoToPointPid go-to-position Controller for Unicycle models
    %   Given the state of a Unicycle vehicle, this Controller computes the
    %   input (i.e., linear and angular velocity) to steer the Unicycle to the
    %   point defined in the constructor. The control input is compute in
    %   a Proportional-Integral-Derivative (PID) fashion.
    %
    %   UniGoToPointPid Properties:
    %
    %   point                    - Desired position in 2-d
    %
    %   UniGoToPointPid Methods:
    %
    %   UniGoToPointPid (point)  - see help UniGoToPointPid.UniGoToPointPid
    %
    % See also Controller, Unicycle, BoxSet
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
    end
    
    methods
        
        function obj = UniGoToPointPid(point,varargin)
            %UniGoToPointPid costructor method
            %
            %   c = UniGoToPointPid(point, par1, val1,...);
            %   where point is the desired position in 2-d and the
            %   parameters are 
            %   
            %   PidGains               - the PID gains of the controller. 
            %                            By default kpid = [1,1,1].
            %
            %   InputBoxSetConstraints -  a BoxSet denoting the input
            %                             constraints.
            %
            %   See also BoxSet
            
            kpid = [1,1,1];
            uSet = BoxSet([-inf;-inf],1:2,[inf;inf],1:2,2);
            
            parameterPointer = 2;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        
                        case 'PidGains'
                            
                            kpid = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'InputBoxSetConstraints'    
                            
                            uSet= varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            obj = obj@DtSystem(...
                'StateEquation',@(xc,z,t)UniGoToPointPid.nextX(xc,z,point), ...
                'OutputEquation',@(xc,z,t)UniGoToPointPid.computeInput(xc,z,point,kpid,uSet), ...
                'nx',2,'nu',3,'ny',2,varargin{:});
            
            
        end
        
    end
    
    methods(Static)
        
        function nextXc = nextX(xc,z,point)
            
            
            R = [cos(z(3)),-sin(z(3));
                sin(z(3)), cos(z(3))];
            
            err = R'*(point - z(1:2));
            
            thetaErr = atan2(err(2),err(1));
            
            
            nextXc = [0,0;1,0]*xc+[1;0]*thetaErr;
            
        end
        
        function u = computeInput(xc,z,point,kpid,uSet)
            
            
            R = [cos(z(3)),-sin(z(3));
                sin(z(3)), cos(z(3))];
            
            err = R'*(point - z(1:2));
            
            thetaErr = atan2(err(2),err(1));
            
            
            v = norm(z(1:2)-point);
            w = kpid(1)*thetaErr+kpid(3)*(thetaErr-xc(1))+kpid(2)*(norm(xc));
            
            u = uSet.project([v;w]);
            %u = [min(max(v,uSet.lowerBounds(1)),uSet.upperBounds(1));
            %     min(max(w,uSet.lowerBounds(2)),uSet.upperBounds(2))];
            
        end
        
    end
    
end