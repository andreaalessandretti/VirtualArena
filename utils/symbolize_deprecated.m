%% symbolize(f,inSizes)
% Example:
%
% f    = @(p1,p2) (p1+p2{1})*(p2{2}'*p2{2});
%
% inSizes = {2,{2,4}};
%
% % i.e., the first input is a vector of dimension [2,1] and the second
% % input is a cell containing two vectors, one of dimension [2,1] and the
% % other of dimension [4,1].
%
% testInput = {[3;3],{[4;4],[5;5;5;5]}};
%
% symf = symbolize(f,inSizes);
%
% out1 = f(inTest{:})
%
% out2 = symf(inTest{:})
%
%

function fout =  symbolize(f,inSizes,fileName)

x = getSymOfCell (inSizes,'x',0);

symf = f(x{:});

if isempty(symf)
    fout = @(varargin)[];
else
    if (nargin == 3 && ischar(fileName))
        fout1 = matlabFunction(symf,'vars',getSymOfCell (inSizes,'x',1),'File',fileName);
    else
        fout1 = matlabFunction(symf,'vars',getSymOfCell (inSizes,'x',1));
    end
    fout = @(varargin)symbolizeEvaluateFunction(fout1,varargin);
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
        xi = getSymOfCell (c{i},[prefix,num2str(i),'_'],expanded);
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


