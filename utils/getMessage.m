%

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

function message = getMessage(MessageCode,varargin)

switch MessageCode
    
    case 'GeneralSystem:LinearizingOutputEquation'
        message = 'Linearizing output equation (symbolic matlab)... ';
        
    case 'GeneralSystem:LinearizingOutputEquationS'
        message = 'Computation linearization via sampling ... ';
        
        
    case 'GeneralSystem:LinearizingStateEquation'
        message = 'Linearizing state equation (symbolic matlab) ... ';
        
    case 'GeneralSystem:LinearizingStateEquationS'
        message = 'Computation linearization via sampling  ... ';
        
    case 'done'
        message = 'done.\n';
        
    case 'MpcOpSolver:LinearizinStageCost'
        
        message = 'Linearizing cost functions ... ';
        
    case 'MpcOpSolver:LinearizingTerminalConstraint'
        
        message = 'Linearizing terminal cotraint ... ';
        
    case 'LinearizationRequired'
        
        message = 'The system needs to be linearized. Use .computeLinearization';
        
    case 'VirtualArena:discretizationStepNotNefined'
        
        message = 'Discretization Step Not Defined';
        
    case 'VirtualArena:UnknownSystemType'
        
        message = 'Unknown system type';
        
    case 'VirtualArena:UnknownControllerType'
        
        message = 'Unknown controller type';
        
    case 'TrackingControllerECC14:wrongvehicle'
        
        message = 'Vehicle not supported this controller.';
        
    case 'VirtualArena:InitObserver'
        
        message = 'State observer not initialized, use parameter ''InitialConditions'' at creation time.';
    
    case 'VirtualArena:InitController'
        
        message = 'Internal state of the controller not initialized, use parameter ''InitialConditions'' at creation time.';
        
    case 'VirtualArena:NotEnoughInitialConditionsController'
        
        message = 'The number of initial conditions of the controller must correspond with the number of initial conditions of the systems.';
    otherwise
        
        message = 'Unknown message.\n';
end
end