
classdef UniGoToPoint < Controller
    %UniGoToPoint go-to-position Controller for Unicycle models
    %   Given the state of a Unicycle vehicle, this Controller computes the
    %   input (i.e., linear and angular velocity) to steer the Unicycle to the
    %   point defined in the constructor
    %
    % UniGoToPoint Properties:
    %
    %   point                 - Desired position in 2-d
    %
    % UniGoToPoint Methods:
    %
    %   UniGoToPoint (point)
    %
    % See also Controller, Unicycle
    % This file is part of VirtualArena.
    
 
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
        
        point; % Destination point
        
    end
    
    methods
        
        
        function obj = UniGoToPoint(point)
            
            obj.point = point;
            
        end
        
        function u = computeInput(obj,x,t)
            
            R = [cos(x(3)),-sin(x(3));
                sin(x(3)), cos(x(3))]; % Body to inertia rotation matrix
            
            err = R'*(obj.point - x(1:2)); % Position error in body frame
            
            thetaErr = atan2(err(2),err(1)); % Angle error
            
            u = [norm(obj.point-x(1:2));
                thetaErr];
            
        end
        
        
    end
    
end