
classdef ReplayMeasuramentsCtSystem < CtSystem & InitDeinitObject
    %ReplayMeasuramentsCtSystem replays measurements previously logged
    %
    %   c = ReplayMeasuramentsCtSystem(...
    %           'TimeLog'       , timeLog,...
    %           'MeasurementLog', controlLog ...
    %           ...
    %       );
    %
    % If the current time is greater that the logged time the system
    % returns the last measurement logged.
 
    properties
    
    timeLog
    measurementLog
    
    end
    
    methods
        
        
        function obj = ReplayMeasuramentsCtSystem (varargin)
            
            obj = obj@CtSystem(varargin{:});
            
            %% Retrive parameters for superclass GeneralSystem
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'TimeLog'
                            
                            obj.timeLog = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'MeasurementLog'
                            
                            obj.measurementLog = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            if isempty(obj.timeLog) || isempty(obj.measurementLog)
                
                error(getMessage('ReplaySys:missingLos'));
                
            end
            
            %% Ask initial Condition
            if isempty(obj.nx)
                obj.nx=1;
            end
           
            if isempty(obj.ny)
                obj.ny=size(obj.measurementLog,1);
            end
            
            obj.initialConditions = zeros(obj.nx,1);
            
            obj.f = @(t,x,u)zeros(obj.nx,1);
            obj.h = @(t,x)obj.readMeasurementLog(t);
            
            
        end
        
        
        function y = readMeasurementLog(obj,t)
            
            indexesMeasurements = find(obj.timeLog>t);
            
            if isempty(indexesMeasurements)
                y = obj.measurementLog(:,end);
            else
            
                indexMeasurement = max(1,indexesMeasurements(1)-1);

                y = obj.measurementLog(:,indexMeasurement);
                
            end
            
        end
        
    end
end
