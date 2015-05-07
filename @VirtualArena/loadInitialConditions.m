
        function loadInitialConditions(obj,iInitialCondition)
            
            nSystems = length(obj.systemsList);
            
            obj.log = cell(1,nSystems);
            
            %% Initialize simulation
            for i = 1:nSystems
                
                
                if iInitialCondition %% Multiple initial conditions
                    
                    if isempty(obj.systemsList{i}.initialConditions{iInitialCondition})
                        error(getMessage('VA:emptyInitialCon'));
                    end
                    
                    % Log initial stateof the system
                    obj.systemsList{i}.x = obj.systemsList{i}.initialConditions{iInitialCondition};
                    
                    % Log initial state of the observer
                    if not(isempty(obj.systemsList{i}.stateObserver))
                        
                        if isempty(obj.systemsList{i}.stateObserver.initialConditions{iInitialCondition})
                            error(getMessage('VA:emptyInitialCon'));
                        end
                        
                        obj.systemsList{i}.stateObserver.x = obj.systemsList{i}.stateObserver.initialConditions{iInitialCondition};
                        
                        
                        
                    end
                    
                    % Log initial state of the controller
                    if isa(obj.systemsList{i}.controller,'CtSystem') || isa(obj.systemsList{i}.controller,'DtSystem')
                        iCons = obj.systemsList{i}.controller.initialConditions;
                        
                        if not(iscell(iCons)) || length(iCons)<iInitialCondition
                            error(getMessage('VirtualArena:NotEnoughInitialConditionController'));
                        end
                        
                        if isempty(obj.systemsList{i}.controller.initialConditions{iInitialCondition})
                            error(getMessage('VA:emptyInitialCon'));
                        end
                        
                        obj.systemsList{i}.controller.x = obj.systemsList{i}.controller.initialConditions{iInitialCondition};
                        
                        if isempty(obj.systemsList{i}.controller.x)
                            error(getMessage('VA:emptyInitialCon'));
                        end
                        
                    end
                    
                else
                    
                    if isempty(obj.systemsList{i}.initialConditions)
                        error(getMessage('VA:emptyInitialCon'));
                    end
                    
                    if not(size(obj.systemsList{i}.initialConditions,1) == obj.systemsList{i}.nx & ...
                            size(obj.systemsList{i}.initialConditions,2) ==1 )
                        
                        error(getMessage('VA:wrongSizeInitialCon'));
                        
                    end
                    
                    % Log the ith initial state of the system
                    obj.systemsList{i}.x = obj.systemsList{i}.initialConditions;
                    
                    % Log initial state of the observer
                    if not(isempty(obj.systemsList{i}.stateObserver))
                        
                        if isempty(obj.systemsList{i}.stateObserver.initialConditions)
                            error(getMessage('VirtualArena:InitObserver'));
                        end
                        
                        obj.systemsList{i}.stateObserver.x = obj.systemsList{i}.stateObserver.initialConditions;
                    end
                    
                    % Log initial state of the controller
                    if isa(obj.systemsList{i}.controller,'CtSystem') || isa(obj.systemsList{i}.controller,'DtSystem')
                        if isempty(obj.systemsList{i}.controller.initialConditions)
                            error(getMessage('VirtualArena:InitController'));
                        end
                        obj.systemsList{i}.controller.x = obj.systemsList{i}.controller.initialConditions;
                    end
                end
                
            end
            
        end
        