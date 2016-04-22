clc;clear all;close all;

N = 5;

for i = 1:N
    
    v{i}                  = CtSystem('StateEquation',@(t,x,u)u,'nx',1,'nu',1);    
    v{i}.controller       = ex5BasicConsensusController();
    v{i}.initialCondition = i;
    
end

%% Network

% Adjacency matrix - loop
A            = zeros(N);
A(1,4)       = 1;
A(2:N,1:N-1) = eye(N-1);

s1 = ex5StateSensor();

%% VirtualArena
a = VirtualArena(v,...
    'StoppingCriteria'  , @(t,as)t>10,...
    'SensorsNetwork'    , {s1,A},...
    'DiscretizationStep', 0.1,...
    'PlottingStep'      , 1);

ret = a.run();

for i=1:length(ret)
    finalState = ret{i}.stateTrajectory(1,end)'
end
