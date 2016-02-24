%% explodePlot explodePlot(logTime,logV,varargin)
% 
% plot(logTime,logV(i,:));
% ylabel('xi'); or ylabel('texti'); with 'text' = varargin{1};
% 
function explodePlot(logTime,logV,varargin)

if nargin == 3
    lbl = varargin{1};
else
    lbl = 'x';
end
nx = size(logV,1);
for i = 1:nx
    subplot(nx,1,i);
    plot(logTime, logV(i,:));
    ylabel(sprintf('%s%i',lbl,i));
    setNicePlot
end

end

