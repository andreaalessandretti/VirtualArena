classdef InlineController < Controller
    %InlineController inline Controller
    %
    %   InlineController peoperties:
    %
    %   law     - @(x,t) function handle of the control law
    %
    %   InlineController methods:
    %
    %   InlineController - constructor, c = InlineController(@law)
    %
    %   See also Controller
    
    % This file is part of VirtualArena.
    %
    % Copyright (C) 2012-14 Andrea Alessandretti
    %
    % andrea.alessandretti@{ist.utl.pt, epfl.ch}
    % Automatic Control Laboratory, EPFL, Lausanne, Switzerland.
    % Institute System and Robotics, IST, Lisbon, Portugal.
    %
    % This program is free software: you can redistribute it and/or modify
    % it under the terms of the GNU General Public License as published by
    % the Free Software Foundation, either version 3 of the License, or
    % (at your option) any later version.
    %
    % This program is distributed in the hope that it will be useful,
    % but WITHOUT ANY WARRANTY; without even the implied warranty of
    % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    % GNU General Public License for more details.
    %
    % You should have received a copy of the GNU General Public License
    % along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    properties
        
    end
    properties (Access=private)
        law = [];
    end
    
    
    methods
        
        
        function obj = InlineController(law)
            
            obj = obj@Controller();
            obj.law = law;
            
        end
        
        
        function u = computeInput(obj,x,t)
            
            u = obj.law(x,t);
            
        end
        
        
    end
    
end