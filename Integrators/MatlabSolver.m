%RK4 is the implementation of the Runge-Kutta method RK4
%
%   See also Integrator, EulerForward



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


classdef MatlabSolver < Integrator
    
    
    properties
        solver;
    end
    
    methods
        function obj = MatlabSolver(nameSolver)
            
            switch nameSolver
                case 'ode45'
                    obj.solver = @ode45;
                case 'ode23'
                    obj.solver = @ode23;
                case 'ode113'
                    obj.solver = @ode113;
                case 'ode15s'
                    obj.solver = @ode15s;
                case 'ode23s'
                    obj.solver = @ode23s;
                case 'ode23t'
                    obj.solver = @ode23t;
                case 'ode23tb'
                    obj.solver = @ode23tb;
                %case 'ode15i'
                %    obj.solver = @ode15i;
                otherwise
                    error(sprintf('Solver %s not supported.',nameSolver));
            end
            
        end
        
        function xkp1 = integrate(obj,f,xk,h)
            
            [t,xx] = obj.solver(@(t,x)f(x),[0,h],xk);
            
            xkp1 = xx(end,:)';
        end
    end
    
end

