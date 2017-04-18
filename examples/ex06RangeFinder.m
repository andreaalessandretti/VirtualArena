%% Example of a Sensor

clc; clear all; close all;

v1 = ICtSystem();
v1.x = [0;0;0];  % Current state. Normally this variable is handled by VirtualArena

v2 = ex06SysWithVolume(BoxSet([-1;-1],[1;1]));
v2.x = [5;5;pi/2];

v3 = ex06SysWithVolume(BoxSet([-1;-1],[1;1]));
v3.x = [-3;3;0];

angles = linspace(0,2*pi,300);
s = RangeFinder(angles,norm([4.5,4.5]));

agentsList = {v1,v2,v3};
detectableAgents = [0,1,1];
ret = s.sense(0,1,agentsList,detectableAgents);

%% Plots

plot(v2.volume+v2.x(1:2)); hold on 
plot(v3.volume+v3.x(1:2)); hold on 

for i = 1:300
    if not(norm(ret(i))==Inf) 
        d = [cos(angles(i));
             sin(angles(i))]*ret(i);
         plot([0,d(1)],[0,d(2)]);
    end
end

