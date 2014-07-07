

function h = stepPlotFunctionPos(agentsList,hist,plot_handles,i)
    
    
        
    if not(plot_handles == 0)
      delete(plot_handles)
    end
    
    if isempty(agentsList{1}.stateObserver)
        
        x = hist{1}.stateTrajectory(:,1:i); hold on;
        
        h = plot(x(1,:),x(2,:));
        
        title('Position')
        
    else
        
        xHat = hist{1}.observerStateTrajectory(:,1:i); hold on;
        
        h = plot(xHat(1,:),xHat(2,:),'--');
        
        title('Estimated Position')
        
    end
    
        
        
       
end
