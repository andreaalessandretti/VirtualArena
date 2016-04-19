function h = ex04StepPlotFunction(sysList,log,plot_handles,k)

logHatX = log{1}.observerStateTrajectory(:,1:k); hold on;
h(1)    = plot(logHatX(1,:),logHatX(2,:),'--');
grid on

end
