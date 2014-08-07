
% Find the largest alpha such that
% {x'Px<=alpha^2} \subseteq {F*x<=f } 
% alpha = getLargestEllipseInPolytope(F,f,P)
% by Andrea Alessandretti 2013
function minAlpha = getLargestEllipseInPolytope(F,f,P)

chP = chol(P);

barF = F/chP;
barf = f;

minAlpha = inf;
for i =1:size(barF,1)
    minAlpha = min(minAlpha,norm(barf(i))/norm(barF(i,:)));
end

end