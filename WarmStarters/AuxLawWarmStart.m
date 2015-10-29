classdef AuxLawWarmStart < WarmStart
    %%WarmStart specifies  what to log during the simulation.
    %
    % See also ShiftAndAppendAuxLawWarmStart,
    % ShiftAndAppendZeroWarmStart, ShiftAndHoldWarmStart, WarmStart
    
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
        auxiliaryLaw;
        nextX;
        dt;
    end

    
    methods
        function obj = AuxLawWarmStart(auxiliaryLaw,nextX,dt)
            obj.auxiliaryLaw = auxiliaryLaw;
            obj.nextX        = nextX;
            obj.dt           = dt;
        end
        
        function sol = generateWarmStarts(obj,t,previousSol)
            
            x_opt = previousSol.x_opt;
            u_opt = previousSol.u_opt;
            kAuc  = obj.auxiliaryLaw;
            
            x0     = previousSol.x_opt(:,2);
            tx_opt = previousSol.tx_opt;
            t      = [tx_opt(2:end),tx_opt(end)+obj.dt];
            
            sol.u = zeros(size(u_opt));
            sol.x = zeros(size(x_opt));
            
            sol.x(:,1)=x0;
            
            for i=2:length(tx_opt)
               sol.u(:,i-1) = kAuc(t(i-1),sol.x(:,i-1));
               sol.x(:,i) = obj.nextX(t(i-1),sol.x(:,i-1),sol.u(:,i-1));
            end
            
            sol.u(:,end)=kAuc(t(end),sol.x(:,end));
        end
     
    end
    
end

