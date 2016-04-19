

function initPlotFunctionMpc()

plot(10*sin(linspace(0,2*pi,100)),10*cos(linspace(0,2*pi,100)),'r--','Linewidth',2);
grid on
axis([5 10 -5 10])
setNicePlot
end
