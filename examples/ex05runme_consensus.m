clc; clear all; close all;

N = 5;

for i = 1:N
    
    v{i}                  = ICtSystem('StateEquation',@(t,x,u,varargin)u,'nx',1,'nu',1);    
    v{i}.controller       = ex05BasicConsensusController();
    v{i}.initialCondition = i;
    
end

%% Network

% Adjacency matrix - loop
A            = zeros(N);
A(1,4)       = 1;
A(2:N,1:N-1) = eye(N-1);

s1 = IAgentSensor(@(t,agentId,agent,sensedAgentId,sensedAgent)sensedAgent.x);

Ah = @(t) A;

%% VirtualArena
a = VirtualArena(v,...
    'StoppingCriteria'  , @(t,as)t>10,...
    'SensorsNetwork'    , {s1,Ah},...
    'DiscretizationStep', 0.1,...
    'PlottingStep'      , 1);

ret = a.run();

for i=1:length(ret)
    finalState = ret{i}.stateTrajectory(1,end)'
end
