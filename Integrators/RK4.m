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


classdef RK4 < Integrator
    
    
    properties
    end
    
    methods (Static)
        function xkp1 = integrate(f,xk,h)
            
            
            k1 = f(xk);
            k2 = f(xk + 0.5*h*k1);
            k3 = f(xk + 0.5*h*k2);
            k4 = f(xk + h*k3);
            
            xkp1 = xk +(1/6)*h*(k1+2*k2+2*k3+k4);
        end
    end
    
end

