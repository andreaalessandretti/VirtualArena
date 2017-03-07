classdef IController < Controller
    %IController inline Controller
    %
    %   IController peoperties:
    %
    %   law     - @(t,x) function handle of the control law
    %
    %   IController methods:
    %
    %   IController - constructor, c = IController(@law)
    %
    %
    % Example:
    %
    % A = [1,2;0,1]; B = [0;1];
    % 
    % [K,P,E] = dlqr(A,B,eye(2),100);
    % 
    % sys = IDtSystem('StateEquation',@(k,x,u)A*x+B*u,'nx',2,'nu',1);
    % 
    % sys.controller = IController(@(k,x)-K*x);
    % 
    % sys.initialCondition = [10;20];
    % 
    % va = VirtualArena(sys,'StoppingCriteria'  ,@(k,as)k>30);
    %  
    % ret = va.run();
    %
    % See also Controller
    
 
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
    properties
        law = [];
    end
    
    
    methods
        
        
        function obj = IController(law)
            
            obj = obj@Controller();
            obj.law = law;
            
        end
        
        
        function u = computeInput(obj,varargin)
            
            u = obj.law(varargin{:});
            
        end
        
        
    end
    
end