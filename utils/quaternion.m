
classdef quaternion
    %quaternion
    %
    % quaternion.q2R
    % quaternion.theta2quat
    % quaternion.q2theta
    % quaternion.R2theta
    %
    
    
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
    
    methods (Static)
        
        function R = q2R(quat)
            x = quat(1);
            y = quat(2);
            z = quat(3);
            w = quat(4);
            
            
            R = [w*w+x*x-y*y-z*z, 2*(-w*z+x*y),    2*(w*y+x*z);
                2*(w*z+x*y),     w*w-x*x+y*y-z*z, 2*(-w*x+y*z);
                2*(-w*y+x*z),    2*(w*x+y*z),     w*w-x*x-y*y+z*z] / (w*w+x*x+y*y+z*z);
            
        end
        
        
        
        function q =  theta2quat(theta)
            
            c1 = cos(theta(1)/2);
            c2 = cos(theta(2)/2);
            c3 = cos(theta(3)/2);
            
            s1 = sin(theta(1)/2);
            s2 = sin(theta(2)/2);
            s3 = sin(theta(3)/2);
            
            
            q = [c1*c2*c3 + s1*s2*s3;
                s1*c2*c3 - c1*s2*s3;
                c1*s2*c3 + s1*c2*s3;
                c1*c2*s3 - s1*s2*c2];
            
        end
        
        function theta = q2theta(q)
            
            theta = quaternion.R2theta(quaternion.q2R(q));
            
        end
        
        
        function theta = R2theta(R)
            theta =[ atan2(R(3,2), R(3,3));
                atan2(-R(3,1), sqrt(R(3,2)*R(3,2) + R(3,3)*R(3,3)));
                atan2(R(2,1), R(1,1))];
        end
        
        
    end
    
end

