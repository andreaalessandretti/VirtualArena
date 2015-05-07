function h = plotSignalPredictionAndEstimate(signal,estimate,prediction,timeSignal,timePrediction)

h = [];

maxColumn = 5;
maxRows   = 6;

nv = size(signal,1);

nColumns = ceil(nv/maxRows);
if nColumns> maxColumn
    return
end
nRows    = ceil(nv/nColumns);

plotEstimate   = ~isempty(estimate);
plotPrediction = ~isempty(prediction);

h = zeros((plotEstimate+plotPrediction+1)*nv,1);
iPlot = 1;

for i=1:nv
    
    subplot(nRows,nColumns,i);
    
    h(iPlot) = plot(timeSignal,signal(i,:)); hold on 
    iPlot = iPlot+1;
    
    if plotEstimate
        h(iPlot) = plot(timeSignal,estimate(i,:),'--'); hold on 
        iPlot = iPlot+1;
    end
    
    if plotPrediction
        h(iPlot) = plot(timePrediction,prediction(i,:),'--'); hold on 
        iPlot = iPlot+1;
    end
    
end


end

function testPlotSignalPredictionAndEstimate()

t = linspace(0,3,100);
tp = linspace(3,4,50);

signal = [sin(t);sin(0.8*t);sin(0.6*t)];
estimate = 0.8*[sin(t);sin(0.8*t);sin(0.6*t)];

prediction = 1.1*[sin(tp);sin(0.8*tp);sin(0.6*tp)];


plotSignalPredictionAndEstimate(signal,estimate,prediction,t,tp)

end
