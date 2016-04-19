function [alphaopt,ret] = va_bisection(varargin)
%% BISECTION (oracle,resolution,LovwerBound,UpperBound,verbose)
%
% ex.
%
% bisection(@(x) x<=3)
%
% tic
% bisection(@(x) x<=3,0.00001)
% toc
%
% tic
% bisection(@(x) x<=3,0.00001,2.8,3.2)
% toc
%
% bisection(@(x) x<=3,0.00001,2.8,3.2,1)

if nargin >=1
    oracle = varargin{1};
    res = 0.001;
    LB  = 0;
    UB  = 10;
    verbose = 0;
    performUbSearch = 1;
    
    
else
    error('Oracle function required.');
end

if nargin >=2 && not(isempty(varargin{2}))
    res = varargin{2};
end

if nargin >=3 && not(isempty(varargin{3}))
    LB = varargin{3};
end

if nargin >=4 && not(isempty(varargin{4}))
    UB = varargin{4};
    performUbSearch = 0;
end

if nargin >=5 && not(isempty(varargin{5}))
    verbose = varargin{5};
end



%% Upper bound search
if performUbSearch
    if verbose
        fprintf('Upper bound search :');
    end
    
    ubFound = false;
    
    nIterations = 0;
    while not(ubFound)
        
        alpha = UB;
        if verbose
            if mod(nIterations,10) == 0
                fprintf('\n%f ',alpha);
            else
                fprintf('%f ',alpha);
            end
        end
        
        goUp = oracle(alpha);
        
        if(goUp)
            UB = UB*2;
        else
            ubFound = true;
        end
        
        nIterations = nIterations+1;
    end
    
    if verbose
        fprintf('\n');
        fprintf('Bisection algorithm :');
    end
end

%% Bisection algorithm

alpha = UB;

nIterations = 0;

while UB-LB > res
    
    alpha = (UB+LB)/2;
    if verbose
        fprintf('UB %d LB %d delta %d (%d) alpha %d\n',UB,LB,UB-LB,res,alpha);
    end
    
    goUp = oracle(alpha);
    
    if(goUp) %No problem detected or umbounded
        
        LB = alpha;
    else
        UB = alpha;
    end
    
    nIterations = nIterations+1;
    
end

alphaopt = (UB+LB)/2;
ret.UB = UB;
ret.LB = LB;
ret.res = res;

end
