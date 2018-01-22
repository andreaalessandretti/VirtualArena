%% explodePlot explodePlot(logTime,logV,varargin)
%plot(logTime,logV(i,:));
%ylabel('xi'); or ylabel('texti'); with 'text' = varargin{1};

function h = explodePlot(logTime,logV,varargin)

nv = size(logV,1);

for i=1:nv
    ylbls{i} = '';
    xlbls{i} = '';
    ttls{i} = '';
end
if nargin==3
    for i=1:nv
        ylbls{i} = sprintf('%s%i',varargin{1},i);
    end
end

parameterPointer = 1;

hasParameters = length(varargin)-parameterPointer>=0;

while hasParameters
    
    if (ischar(varargin{parameterPointer}))
        
        switch varargin{parameterPointer}
            
            case 'xlabel'
                
                xlblsIn = varargin{parameterPointer+1};
                if iscell(xlblsIn)
                    xlbls=xlblsIn;
                elseif ischar(xlblsIn)
                    
                    for i=1:nv
                        xlbls{i} = xlblsIn;
                    end
                else
                    error('Format of xlabel not valid');
                end
                
                parameterPointer = parameterPointer+2;
                
            case 'ylabel'
                
                ylbls = varargin{parameterPointer+1};
                
                parameterPointer = parameterPointer+2;
                
            case 'title'
                
                ttlsIn = varargin{parameterPointer+1};
                
                
                if iscell(ttlsIn)
                    ttls=ttlsIn;
                elseif ischar(xlblsIn)
                    
                        ttls{1} = ttlsIn;
                    
                else
                    error('Format of xlabel not valid');
                end
                
                parameterPointer = parameterPointer+2;
                
            otherwise
                
                parameterPointer = parameterPointer+1;
        end
    else
        parameterPointer = parameterPointer+1;
    end
    
    hasParameters = length(varargin)-parameterPointer>=0;
    
end


nx = size(logV,1);
h = zeros(nx);

for i = 1:nx
    subplot(nx,1,i);
    h(i)=plot(logTime, logV(i,:));
    ylabel(ylbls{i});
    xlabel(xlbls{i});
    title(ttls{i});
    setNicePlot
end

end

