
%%DtSystem discrete-time system
%
% Consider a discrete-time dynamical model described by
%
% x+ = f(t,x,u)
% y  = h(t,x)/h(t,x,u)
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
% See also GeneralSystem, DtSystem



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




classdef DtSystem < GeneralSystem
    
    
    methods
        
        function obj = DtSystem (varargin)
            %A discrete time system can be created as
            %
            %   dtSys = DtSystem(par1,val1,par2,val2,...)
            %
            %   where the parameters and the associated values are specified in
            %   the help of the abstract class GeneralSystem, or as
            %   discretization of a continuous time system as
            %
            %   dtSys = DtSystem(ctSys,dt)
            %   dtSys = DtSystem(ctSys,dt,integrator)
            %
            %   where ctSys is of the class CtSystem, dt is the discretization
            %   step and integrator is an integration method (Default RK4)
            %
            %   See also GeneralSystem, CtSystem, Integrator, RK4
            
            
            if ( (nargin == 3 || nargin == 2) && isa(varargin{1},'CtSystem') )
                
                ctSys = varargin{1};
                
                superClassParameters = ctSys.getParameters();
                
                dt = varargin{2};
                
                indexF = find(strcmp(superClassParameters, 'StateEquation'));
                
                if nargin == 3
                    superClassParameters{indexF +1} = @(k,x,u) varargin{3}.integrate(@(y)ctSys.f(dt*k,y,u),x,dt);
                else
                    superClassParameters{indexF +1} = @(k,x,u) RK4.integrate(@(y)ctSys.f(dt*k,y,u),x,dt);
                end
                
                %superClassParameters{indexF +1} = @(x,u) RK4.integrate(@(y)ctSys.f(y,u),x,dt);
                
            else
                
                superClassParameters = varargin;
                
            end
            
            obj = obj@GeneralSystem(superClassParameters{:});
            
        end
        
        
        function x = getStateTrajectory(obj,k0,x0,u)
            
            x = zeros(length(x0),size(u,2)+1);
            x(:,1) = x0;
            k = k0;
            for i =1:size(u,2)
                x(:,i+1) = obj.f(k,x(:,i),u(:,i));
                k = k+1;
            end
            
            if sum(sum(isnan(x)))>0 || sum(sum(isinf(x)))>0
                error('inf or nan prediction');
            end
            
        end
        
        
    end
end