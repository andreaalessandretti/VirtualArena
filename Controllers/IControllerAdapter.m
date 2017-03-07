
classdef  IControllerAdapter < ControllerAdapter
    %
    % ControllerAdapter(originalController)
    %
    % Abstract method:
    %
    % uOriginal2newU(obj,t,x,uOriginal)
    %
    
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
        
        originalU2newUFnc;
        x2originalXFnc;
    end
    
    
    methods
        
        %%
        %
        % IControllerAdapter(originalU2newU,originalController,x2originalX)
        % 
        % originalX = x2originalX(obj,t,x);
        % originalU = originalController.computeInput(t,originalX);
        % u         = originalU2newU(obj,t,x,originalU);
        %
        %
        function obj = IControllerAdapter(originalU2newU,originalController,x2originalX)
            
            obj = obj@ControllerAdapter(originalController);
            
            obj.originalU2newUFnc = originalU2newU;
            obj.x2originalXFnc    = x2originalX;
            
        end
        
        function  uNew = originalU2newU(obj,t,x,uOriginal)
            uNew = obj.originalU2newUFnc(t,x,uOriginal);
        end
        
        function  originalX = x2originalX(obj,t,x)
            originalX = obj.x2originalXFnc(t,x);
        end
        
    end
end