classdef Symbolizer
    methods (Static)
        function fout =  symbolize(f,inSizes,fileName)
            
            x = Symbolizer.getSymOfCell (inSizes,'x',0);
            
            symf = f(x{:});
            
            if isempty(symf)
                fout = @(varargin)[];
            else
                if (nargin == 3 && ischar(fileName))
                    fout1 = matlabFunction(symf,'vars',Symbolizer.getSymOfCell (inSizes,'x',1),'File',fileName);
                else
                    fout1 = matlabFunction(symf,'vars',Symbolizer.getSymOfCell (inSizes,'x',1));
                end
                fout = @(varargin)Symbolizer.symbolizeEvaluateFunction(fout1,varargin);
            end
        end
        
        
        function symCell = getSymOfCell (c,prefix,expanded)
            
            nInpuuts = length(c);
            symCell = {};
            
            for i = 1:nInpuuts
                if isnumeric(c{i}) % base case
                    xi = sym([prefix,num2str(i)],[c{i},1]);
                    if isempty(which('assume'))
                        xi = sym(xi,'real');
                    else
                        assume(xi,'real');
                    end
                    symCell = {symCell{:},xi};
                elseif iscell(c{i})
                    xi = Symbolizer.getSymOfCell (c{i},[prefix,num2str(i),'_'],expanded);
                    if expanded
                        symCell = {symCell{:},xi{:}};
                    else
                        symCell = {symCell{:},xi};
                    end
                else
                    error('Error: check the help for the correct use of the function.');
                end
                
                
            end
            
            
        end


        function ret = symbolizeEvaluateFunction(f,inputs)
            expInp = Symbolizer.expandInputs (inputs);
            ret    = f(expInp{:});
        end

function symCell = expandInputs (in)

nInpuuts = length(in);
symCell = {};

for i = 1:nInpuuts
    if iscell(in{i})
        xi = Symbolizer.expandInputs (in{i});
        symCell = {symCell{:},xi{:}};
        
    else % base case
        symCell = {symCell{:},in{i}};
            %error('Error: check the help for the correct use of the function.');
    end
    
    
end

end


    end
end


