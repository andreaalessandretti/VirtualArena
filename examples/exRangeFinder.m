%% Example of a Sensor

clc;clear all;close all;

v1 = Unicycle();
v1.x = [0;0;0]; % Current state. Normally this variable is handled by VirtualArena

v2 = Unicycle('Occupancy',BoxSet([-1;-1],1:2,[1;1],1:2,2));
v2.x = [5;5;0];

v3 = Unicycle('Occupancy',BoxSet([-1;-1],1:2,[1;1],1:2,2));
v3.x = [-3;3;0];

angles = linspace(0,2*pi,300);
s = RangeFinder(angles,norm([4.5,4.5]));

agentsList = {v1,v2,v3};
detectableAgents = [0,1,1];
ret = s.sense(1,agentsList,detectableAgents);

%% Plots

plot(v2.occupancy+v2.x(1:2)); hold on 
plot(v3.occupancy+v3.x(1:2)); hold on 

for i = 1:300
    if not(norm(ret(i))==Inf) 
        d = [cos(angles(i));
             sin(angles(i))]*ret(i);
         plot([0,d(1)],[0,d(2)]);
    end
end

