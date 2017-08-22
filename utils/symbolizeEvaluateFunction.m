
function ret = symbolizeEvaluateFunction(f,inputs)
expInp = expandInputs (inputs);
ret    = f(expInp{:});
end

function symCell = expandInputs (in)

nInpuuts = length(in);
symCell = {};

for i = 1:nInpuuts
    if isnumeric(in{i}) % base case
        
        symCell = {symCell{:},in{i}};
        
    elseif iscell(in{i})
        xi = expandInputs (in{i});
        symCell = {symCell{:},xi{:}};
        
    else
            error('Error: check the help for the correct use of the function.');
    end
    
    
end

end