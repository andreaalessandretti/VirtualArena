function h = ex01StepPlotFunction(sysList,log,plot_handles,k)

logX = log{1}.stateTrajectory(:,1:k); hold on;
h    = plot(logX(1,:),logX(2,:));
grid on

end
