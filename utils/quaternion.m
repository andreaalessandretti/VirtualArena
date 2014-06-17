
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

