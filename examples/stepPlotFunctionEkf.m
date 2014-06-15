

function h = stepPlotFunctionEkf(agentsList,hist,plot_handles,i)
    
    if not(plot_handles == 0)
      delete(plot_handles)
    end
    
    nAgents = length(agentsList);
    h = zeros(1,nAgents);
    
    for k = 1:nAgents
        
        
        x = hist{k}.stateTrajectory(:,1:i); hold on;
        
        h((k-1)*2+1) = plot(x(1,:),x(2,:));
        
        xHat = hist{k}.observerStateTrajectory(:,1:i); hold on;
        
        h((k-1)*2+2) = plot(xHat(1,:),xHat(2,:),'--');
        
    end
    
    grid on
    
end
