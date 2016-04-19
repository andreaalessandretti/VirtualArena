function h = ex02StepPlotFunction(sysList,log,plot_handles,k)

logX    = log{1}.stateTrajectory(:,1:k); hold on;
logHatX = log{1}.observerStateTrajectory(:,1:k); hold on;
h(1)    = plot(logX(1,:)   ,logX(2,:));
h(2)    = plot(logHatX(1,:),logHatX(2,:),'--');
grid on

end
