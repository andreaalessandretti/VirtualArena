classdef AuxLawWarmStart < WarmStart
    %%AuxLawWarmStart
    
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
        dt;
    end
    
    methods (Abstract)
        auxiliaryLaw(obj,t,x);
        nextX(obj,t,x,u);
    end

    
    methods
        function obj = AuxLawWarmStart(dt)
            obj.dt           = dt;
        end
        
        function sol = generateWarmStarts(obj,t,previousSol)
            
            x_opt = previousSol.x_opt;
            u_opt = previousSol.u_opt;
            
            x0     = previousSol.x_opt(:,2);
            tx_opt = previousSol.tx_opt;
            t      = [tx_opt(2:end),tx_opt(end)+obj.dt];
            
            sol.u_opt  = zeros(size(u_opt));
            sol.x_opt  = zeros(size(x_opt));
            sol.tx_opt = t;
            
            sol.x_opt(:,1)=x0;
            
            for i=2:length(tx_opt)
               sol.u_opt(:,i-1) = obj.auxiliaryLaw(t(i-1),sol.x_opt(:,i-1));
               sol.x_opt(:,i) = obj.nextX(t(i-1),sol.x_opt(:,i-1),sol.u_opt(:,i-1));
            end
            
        end
     
    end
    
end

