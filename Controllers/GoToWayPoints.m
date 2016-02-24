

classdef GoToWayPoints < DtSystem
%%GoToWayPoints WayPoints controller for Unicycle models
% 
% GoToWayPoints(points,par1,val1,par2,val2,...)
% 
% Control Input:
%
% v = norm(currentPosition-pointPosition); %Linear Velocity
%
% w = kpid(1)*thetaErr(k) + ...
%     kpid(2)*(thetaErr(k)+thetaErr(k-1)) + ...
%     kpid(3)*(thetaErr(k)-thetaErr(k-1)); %Angular Velocity
%
% u = [v;w] projected into 'InputBoxSetConstraints'
%
% Parameters:
%
% 'PidGains',               Vector kpid (Default [10,0,0])
% 'InputBoxSetConstraints'  Contraint set (Default BoxSet([-inf;-inf],[inf;inf]) );
% 'DetectionRadius'         A waypoint is reached when the vehicle is closer than
%                           DetectionRadius meters (Default 2)
%
% Example
% -----------------------------------------------------
% clc; close all; clear all;
%
% sys = CtSystem('StateEquation',@(t,x,u) [
%    cos(x(3))*u(1);
%    sin(x(3))*u(1);
%    u(2)],'nx',3, 'nu',2);
%
% sys.initialConditions = [0;0;pi/2];
%
% points = [ 0,0; 0,15; 5,15; 5,5; 15,5; 15,2; %L
%            20,2;  30,15; 40,0; 35,0; 33,4; 27,4; 20,0]';
%
% plot(points(1,:),points(2,:),'o'); hold on
%
% sys.controller = GoToWayPoints(points,'DetectionRadius',1, 'InputBoxSetConstraints',BoxSet([0;-pi/3],[2;pi/3]));
%
% va = VirtualArena(sys,...
%     'StoppingCriteria',@(t,as)t>80,...
%     'StepPlotFunction', @(systemsList,log,oldHandles,k) plot(log{1}.stateTrajectory(1,1:k),log{1}.stateTrajectory(2,1:k)),... @stepPoltTrajecotry, ...
%     'DiscretizationStep',0.1);
%
% ret = va.run();

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
        currentPointIndex
        points
        radius = 2;
    end
    
    methods
        
        function obj = GoToWayPoints(points,varargin)
            
            obj = obj@DtSystem( 'nx',2,'nu',3,'ny',2,'InitialConditions',[0;0]);
            
            %% Default Values
            
            kpid = [10,0,0];
            
            uSet = BoxSet([-inf;-inf],[inf;inf]);
            
            obj.points = points;
            
            obj.currentPointIndex = 1;
            
            %% Load Parameters
            
            parameterPointer = 1;
            
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
                            
                        case 'DetectionRadius'    
                            
                            obj.radius = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            
            obj.f = @(t,xc,u,z)obj.nextX(xc,z);
            
            obj.h = @(t,xc,z)  obj.computeInput(xc,z,kpid,uSet);
            
            
            
        end
        
        function nextXc = nextX(obj,xc,z)
            
            pntIndex = obj.currentPointIndex; 
            point    = obj.points(:,pntIndex);
            
            R = [cos(z(3)),-sin(z(3));
                 sin(z(3)), cos(z(3))];
            
            err = R'*(point - z(1:2));
            
            thetaErr = atan2(err(2),err(1));
            
            nextXc = [0,0;
                      1,0]*xc + [1;0]*thetaErr;
            
        end
        
        function u = computeInput(obj,xc,z,kpid,uSet)
            
            
            
            pntIndex = obj.currentPointIndex;
            
            point    = obj.points(:,pntIndex);
            
            %% Switch Point if close
            
            if norm(point - z(1:2))< obj.radius
                
                if pntIndex == length(obj.points)
                    pntIndex = 1;
                else
                    pntIndex = pntIndex+1;
                end
                
                point = obj.points(:,pntIndex);
                
                obj.currentPointIndex = pntIndex;
                
            end
            
            
            %% Compute Errors
            
            R = [cos(z(3)),-sin(z(3));
                sin(z(3)), cos(z(3))];
            
            err = R'*(point - z(1:2));
            
            thetaErr = atan2(err(2),err(1));
            
            %% Compute Inputs
            
            v = norm(z(1:2)-point);
            
            w = kpid(1)*thetaErr+kpid(3)*(thetaErr-xc(1))+kpid(2)*(thetaErr+xc(1));
            
            u = uSet.project([v;w]);
            
            
        end
        
    end
    
end