function setOptions(obj,varargin)
            
            if isa(varargin{1},'cell')
                obj.systemsList = varargin{1};
            else
                obj.systemsList = {varargin{1}};
            end
            
            parameterPointer = 2;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'StoppingCriteria'
                            
                            obj.stoppingCriteria = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StepPlotFunction'
                            
                            obj.stepPlotFunction = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'InitialTime'
                            
                            obj.initialTime = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'DefaultPlots'
                            
                            obj.defaultPlots = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'InitPlotFunction'
                            
                            obj.initPlotFunction = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StopPlotFunction'
                            
                            obj.stopPlotFunction = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'DisplaySelector'
                            obj.display =varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'HandlePostFirstPlot'
                            
                            obj.handlePostFirstPlot =varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'VideoName'
                            
                            obj.videoName =varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'DiscretizationStep'
                            
                            obj.discretizationStep = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'SensorsNetwork'
                            
                            obj.sensorsNetwork = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'PlottingStep'
                            
                            obj.plottingStep = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Integrator'
                            
                            obj.integrator = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'ExtraLogs'
                            
                            extraLogs   = varargin{parameterPointer+1};
                            if iscell(extraLogs)
                                obj.logObjs = {obj.logObjs{:},extraLogs{:}};
                            else
                                obj.logObjs = {obj.logObjs{:},extraLogs};
                            end
                            parameterPointer = parameterPointer+2;
                            
                        case 'MultiRuns'
                            
                            obj.multiruns = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'RealTime'
                            
                            obj.realTime = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
end
        
