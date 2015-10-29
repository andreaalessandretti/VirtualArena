%% save2pdf2
%  'FileName','Units','Handle','RestoreSettings', 'dpi', 'DimPlot', 'SaveFile'
%
% e.g., save2pdf2('FileName','testtt','DimPlot',[10,7],'CorrectionPaperPosition',[[-0.5, -0.5, 0.5, 0.5]])
function save2pdf2(varargin)

handle = gcf;
dpi = 150;
units = 'centimeters';
predefinedFileName = 0;
dimPlot = [];
parameterPointer = 1;
saveFile = 1;
hasParameters = length(varargin)-parameterPointer>=0;
correctionPaperPosition = zeros(1,4);
restoreSettings = 1;
while hasParameters
    
    if (ischar(varargin{parameterPointer}))
        
        switch varargin{parameterPointer}
            
            case 'FileName'
                
                fileName = varargin{parameterPointer+1};
                
                predefinedFileName = 1;
                
                parameterPointer = parameterPointer+2;
                
            case 'Units'
                
                units = varargin{parameterPointer+1};
                
                parameterPointer = parameterPointer+2;
                
            case 'Handle'
                
                handle = varargin{parameterPointer+1};
                
                parameterPointer = parameterPointer+2;
                
            case 'RestoreSettings'
                
                restoreSettings = varargin{parameterPointer+1};
                
                parameterPointer = parameterPointer+2;
                
            case 'dpi'
                
                dpi = varargin{parameterPointer+1};
                
                parameterPointer = parameterPointer+2;
                
            case 'DimPlot'
                
                dimPlot = varargin{parameterPointer+1};
                
                parameterPointer = parameterPointer+2;
                
            case 'SaveFile'
                
                saveFile = varargin{parameterPointer+1};
                
                parameterPointer = parameterPointer+2;
                
            case 'CorrectionPaperPosition'
                
                correctionPaperPosition = varargin{parameterPointer+1};
                
                parameterPointer = parameterPointer+2;
                
            otherwise
                
                parameterPointer = parameterPointer+1;
                
        end
    else
        parameterPointer = parameterPointer+1;
    end
    
    hasParameters = length(varargin)-parameterPointer>=0;
    
end


if not(predefinedFileName) && saveFile
    [fileName,pathName] = uiputfile('*.pdf','Save to PDF file:');
    if fileName == 0; return; end
    fileName = [pathName,fileName];
end


if restoreSettings
    
    prePaperType     = get(handle,'PaperType');
    prePaperUnits    = get(handle,'PaperUnits');
    preUnits         = get(handle,'Units');
    prePaperPosition = get(handle,'PaperPosition');
    prePaperSize     = get(handle,'PaperSize');
    
end

% Set units to all be the same
set(handle,'PaperUnits',units);
set(handle,'Units',units);

position = get(handle,'Position');

if isempty(dimPlot)
    dimPlot =position(3:4);
end

set(handle,'PaperPosition',[0,0,dimPlot]+correctionPaperPosition);
set(handle,'PaperSize',dimPlot);


if saveFile
    print(handle,'-dpdf',fileName,sprintf('-r%d',dpi))
end


if restoreSettings
    set(handle,'PaperType'    ,prePaperType);
    set(handle,'PaperUnits'   ,prePaperUnits);
    set(handle,'Units'        ,preUnits);
    set(handle,'PaperPosition',prePaperPosition);
    set(handle,'PaperSize'    ,prePaperSize);
end


end
