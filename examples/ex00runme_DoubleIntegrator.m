%% Introducing Virtual Arena

clc;close all;clear all;

sys = CtSystem(...
    'StateEquation', @(t,x,u) [0,1;0,0]*x+[0;1]*u,...
    'nx',2,'nu',1 ...
);

sys.initialCondition = [1;1];

sys.controller = InlineController(@(t,x)-[1,1.7321]*x);

va = VirtualArena(sys,...
    'DiscretizationStep',0.1...
    );

log = va.run();

log{1}