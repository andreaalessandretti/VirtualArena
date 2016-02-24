
        function loadInitialConditions(obj,iInitialCondition)
            
            nSystems = length(obj.systemsList);
            
            obj.log = cell(1,nSystems);
            
            %% Initialize simulation
            for i = 1:nSystems
                
                
                if iInitialCondition %% Multiple initial conditions
                    
                    if isempty(obj.systemsList{i}.initialCondition{iInitialCondition})
                        error(getMessage('VA:emptyInitialCon'));
                    end
                    
                    % Log initial stateof the system
                    obj.systemsList{i}.x = obj.systemsList{i}.initialCondition{iInitialCondition};
                    
                    % Log initial state of the observer
                    if not(isempty(obj.systemsList{i}.stateObserver))
                        
                        if isempty(obj.systemsList{i}.stateObserver.initialCondition{iInitialCondition})
                            error(getMessage('VA:emptyInitialCon'));
                        end
                        
                        obj.systemsList{i}.stateObserver.x = obj.systemsList{i}.stateObserver.initialCondition{iInitialCondition};
                        
                        
                        
                    end
                    
                    % Log initial state of the controller
                    if isa(obj.systemsList{i}.controller,'CtSystem') || isa(obj.systemsList{i}.controller,'DtSystem')
                        iCons = obj.systemsList{i}.controller.initialCondition;
                        
                        if not(iscell(iCons)) || length(iCons)<iInitialCondition
                            error(getMessage('VirtualArena:NotEnoughInitialConditionController'));
                        end
                        
                        if isempty(obj.systemsList{i}.controller.initialCondition{iInitialCondition})
                            error(getMessage('VA:emptyInitialCon'));
                        end
                        
                        obj.systemsList{i}.controller.x = obj.systemsList{i}.controller.initialCondition{iInitialCondition};
                        
                        if isempty(obj.systemsList{i}.controller.x)
                            error(getMessage('VA:emptyInitialCon'));
                        end
                        
                    end
                    
                else
                    
                    if isempty(obj.systemsList{i}.initialCondition)
                        error(getMessage('VA:emptyInitialCon'));
                    end
                    
                    if not(size(obj.systemsList{i}.initialCondition,1) == obj.systemsList{i}.nx & ...
                            size(obj.systemsList{i}.initialCondition,2) ==1 )
                        
                        error(getMessage('VA:wrongSizeInitialCon'));
                        
                    end
                    
                    % Log the ith initial state of the system
                    obj.systemsList{i}.x = obj.systemsList{i}.initialCondition;
                    
                    % Log initial state of the observer
                    if not(isempty(obj.systemsList{i}.stateObserver))
                        
                        if isempty(obj.systemsList{i}.stateObserver.initialCondition)
                            error(getMessage('VirtualArena:InitObserver'));
                        end
                        
                        obj.systemsList{i}.stateObserver.x = obj.systemsList{i}.stateObserver.initialCondition;
                    end
                    
                    % Log initial state of the controller
                    if isa(obj.systemsList{i}.controller,'CtSystem') || isa(obj.systemsList{i}.controller,'DtSystem')
                        if isempty(obj.systemsList{i}.controller.initialCondition)
                            error(getMessage('VirtualArena:InitController'));
                        end
                        obj.systemsList{i}.controller.x = obj.systemsList{i}.controller.initialCondition;
                    end
                end
                
            end
            
        end
        