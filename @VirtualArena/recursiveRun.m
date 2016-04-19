
        function logAllAll = recursiveRun(obj,multirunsToDo)
            
            if length(multirunsToDo) == 1 %% base of the recursion
                
                multirunToDo = multirunsToDo{1};
                
                for j = 1:multirunToDo.n
                    multirunToDo.initFun(obj,j);
                    
                    if isa(obj.systemsList{1}.initialCondition,'cell')
                        
                        nInitialConditoins = length(obj.systemsList{1}.initialCondition);
                        logAll = cell(1,nInitialConditoins);
                        
                        for i = 1:nInitialConditoins
                            
                            if obj.display.recursiveRun
                                
                                fprintf('Multirun %i, Initial Condition %i\n',j,i);
                                
                            end
                            
                            logAll{i} = obj.runOne(i);
                        end
                        
                        if isa(obj.stopPlotFunction,'function_handle')
                            obj.stopPlotFunction(logAll,obj); hold on
                        end
                        
                    else
                        
                        logAll = obj.runOne();
                        
                        if isa(obj.stopPlotFunction,'function_handle')
                            
                            obj.stopPlotFunction(logAll,obj); hold on
                        end
                        
                    end
                    
                    logAllAll{j} = logAll;
                    
                end
                
                if multirunToDo.n == 1
                    logAllAll = logAll;
                end
                
                
            else % recursion
                
                multirunToDo = multirunsToDo{1};
                
                multirunNow         = multirunToDo{end};
                multirunToRecursion = multirunToDo{1:end-1};
                
                for i = 1:multirunNow.n
                    
                    multirunNow{i}.initFun(obj,i);
                    
                    logAllAll{i}= obj.recursiveRun(multirunToRecursion);
                end
            end
            
        end