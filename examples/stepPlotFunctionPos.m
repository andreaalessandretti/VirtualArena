

function h = stepPlotFunctionPos(agentsList,hist,plot_handles,i)
    
    
        
    if not(plot_handles == 0)
      delete(plot_handles)
    end
    
    nAgents = length(agentsList);
    h = zeros(1,nAgents);
    
    for k = 1:nAgents
        
        x = hist{k}.stateTrajectory(:,1:i); hold on;
        
        if isa(agentsList{k},'Unicycle')
            h(k) = plot(x(1,:),x(2,:));
        else
            h(k) = plot3(x(1,:),x(2,:),x(3,:));
        end
        
    end
    grid on 
end
