%Vehicle
%
% Vehicle Properties:
%
% n          - position space dimension (2-D or 3-D)
% x          - current state vector
% controller - controller used to drive the vehicle
% occupancy  - PolytopicSet representing occupancy of the vehicle in 2D
%
% Costructor, Vehicle(par1,val1,...) :
% 
% Parameters : 'PositionSpaceDimension',  'Controller',  'Occupancy'
%
% See also CtSystem, PolytopicSet


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


classdef Vehicle < CtSystem
    %Vehicle
    
    properties
        n  = [];          %position space dimension (2-D or 3-D)
        occupancy = [];  %PolytopicSet representing occupancy of the vehicle in 2D
    end
    
    methods
        
        function obj = Vehicle(varargin)
     
           obj = obj@CtSystem(varargin{:});
            
           
           parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'PositionSpaceDimension'
                            
                            obj.n = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                        
                            
                       case 'Occupancy'
                            
                            obj.occupancy = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            if isempty(obj.n)
                error ('Sepcify PositionSpaceDimension')
            end
            
        end
        
       
    end
    
end