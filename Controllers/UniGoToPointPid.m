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
        point
        kpid = [1,1,1];
        uSet = BoxSet([-inf;-inf],1:2,[inf;inf],1:2,2);
    end
    
    methods
        
        function obj = UniGoToPointPid(point,varargin)
            %UniGoToPointPid constructor method
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
            
            obj = obj@DtSystem('nx',2,'nu',3,'ny',2,varargin{:});
            obj.point = point;
            
        end
        
        function nextXc = f(obj,t,xc,uSysCon,z,varargin)
            
            point = obj.point;
            
            R = [cos(z(3)),-sin(z(3));
                 sin(z(3)), cos(z(3))];
            
            err = R'*(point - z(1:2));
            
            thetaErr = atan2(err(2),err(1));
            
            
            nextXc = [0,0;
                      1,0]*xc + [1;0]*thetaErr;
                  
        end
        
        function u = h(obj,t,xc,z,varargin)
            
            point = obj.point;
            kpid  = obj.kpid;
            uSet  = obj.uSet;
            
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