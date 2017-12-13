
classdef CtSystem < DynamicalSystem
    %%DynamicalSystem
    %
    % Consider a continuous-time dynamical model described by
    %
    % \dot{x} = f(t,x,u)
    % y       = h(t,x)/h(t,x,u)
    % 
    % where
    %
    % x is an nx-dimensional vector
    % u is an nu-dimensional vector
    % y is an ny-dimensional vector
    % 
    % In VA such system is defined as follows
    % 
    % sys = CtSystem(par1,val1,par2,val2,...)
    %
    % where the parameters are chosen among
    %
    % 'StateEquation', 'nx', 'nu', 'OutputEquation', 'ny' , 
    % 'InitialCondition', 'Controller','StateObserver'
    %
    % Often, it is also possible to set the parameters after the creation
    % of the object, e.g.,
    %
    % sys.controller = mycontroller;
    % sys.stateObserver = myobserver;
    % sys.initialCondition = [1;1];
    %
    % Some methods provided by this function are the following:
    %
    %   getStateTrajectory - Compute a solution of the system. 
    %                        See help CtSystem.getStateTrajectory.
    %   changeOfCoordinate - Perform a change of state and/or input
    %                        coordinate.
    %                        See help CtSystem.changeOfCoordinate.
    %
    % See also DynamicalSystem, DtSystem
    
 
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
    
    
    methods
        
        
        function obj = CtSystem (varargin)
            
            obj = obj@DynamicalSystem(varargin{:});
            
        end
        
        
        
        function x = getStateTrajectory(obj,t0,x0,u,dt,varargin)
            %getStateTrajectory Compute a solution
            %
            % Returns the state trajectory with the form [x0,x1,...]
            % obtained by solving recursively
            %
            % x(:,i+1) = RK4.integrate(@(y)obj.f(t,y,u(:,i)),x(:,i),dt); 
            %     t = t + dt;
            % Calling the function:
            %
            % sys.getStateTrajectory(x0,u,dt)
            
            x = zeros(length(x0),size(u,2)+1);
            x(:,1) = x0;
            t = t0;
            for i =1:size(u,2)
                if nargin ==6
                   x(:,i+1) = varargin{1}.integrate(@(y)obj.f(t,y,u(:,i)),x(:,i),dt); 
                  t = t + dt;
                else
                   x(:,i+1) = RK4.integrate(@(y)obj.f(t,y,u(:,i)),x(:,i),dt); 
                  t = t + dt;
                end
                 
            end
            
            if sum(sum(isnan(x)))>0 || sum(sum(isinf(x)))>0
                error('The solution contains inf or nan elements');
            end
            
        end
        
        function x = getStateTrajectoryLaw(obj,t0,x0,law,dt,N)
            %getStateTrajectory Compute a solution
            %
            % Returns the state trajectory with the form [x0,x1,...]
            % obtained by solving recursively
            %
            % x(:,i+1) = RK4.integrate(@(y)obj.f(t,y,u(:,i)),x(:,i),dt); 
            %     t = t + dt;
            % Calling the function:
            %
            % sys.getStateTrajectory(x0,u,dt)
            
            x = zeros(length(x0),N+1);
            x(:,1) = x0;
            t = t0;
            for i =1:N
                 x(:,i+1) = RK4.integrate(@(y)obj.f(t,y,law(t,x(:,i))),x(:,i),dt); 
                 t = t + dt;
            end
            
            if sum(sum(isnan(x)))>0 || sum(sum(isinf(x)))>0
                error('The solution contains inf or nan elements');
            end
            
        end
        
        function [t,x,u] = getTrajectories(obj,t0,x0,law,dt,N,nu)
            %getStateTrajectory Compute a solution
            %
            % Returns the state trajectory with the form [x0,x1,...]
            % obtained by solving recursively
            %
            % x(:,i+1) = RK4.integrate(@(y)obj.f(t,y,u(:,i)),x(:,i),dt); 
            %     t = t + dt;
            % Calling the function:
            %
            % sys.getStateTrajectory(x0,u,dt)
            t = zeros(1,N+1);
            x = zeros(length(x0),N+1);
            u = zeros(nu,N);
            x(:,1) = x0;
            t(1) = t0;
            for i =1:N
                 u(:,i)   = law(t(i),x(:,i));
                 x(:,i+1) = RK4.integrate(@(y)obj.f(t(i),y,u(:,i)),x(:,i),dt); 
                 t(i+1)   = t(i) + dt;
            end
            
            if sum(sum(isnan(x)))>0 || sum(sum(isinf(x)))>0
                error('The solution contains inf or nan elements');
            end
            
        end
        
        
        
        
    end
end