

function h = stepPlotFunctionMpc(agentsList,hist,plot_handles,i)
    
    if not(plot_handles == 0)
      delete(plot_handles)
    end
    
    nAgents = length(agentsList);
    h = zeros(1,nAgents);
    
    indexPlots = 1;
    
    for k = 1:nAgents
        
        x = hist{k}.stateTrajectory(:,1:i-1); hold on;
        
        h(indexPlots) = plot(x(1,:),x(2,:));
        indexPlots=indexPlots+1;
        
        if isa(agentsList{k}.controller,'MpcController')
            x_opt = agentsList{k}.controller.lastSolution.x_opt;
            
            h(indexPlots) = plot(x_opt(1,:),x_opt(2,:),'--');hold on;
            indexPlots=indexPlots+1;
        
        end
        
    end
    grid on 
end
