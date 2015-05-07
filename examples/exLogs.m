%By default VA logs the state vector, the input vector, and the time. 
%Moreover, it logs the internal state of dynamic controllers state estimate
%of state observers, if any. 
%
%Personalized logger can be created and given to VA with the parameter 
%'ExtraLogs', see help VirtualArena.
%
%For any log, one can down sample the number of logs by defining the 
%log.deltaStep OR log. deltaTime. In this case VA will log the date when 
%mod(step, deltaStep)== 0 or mod(time, deltaTime)== 0, respectively.

tic 
ex00VirtualArena
t1 = toc;

ret{1}{1}.time % show time log of the first vehicle from the first initial condition
va.logObjs     % show logs

%va.logObjs{1}.deltaStep = 10;
%va.logObjs{4}.deltaTime = 1;

tic 
ret2 = va.run();
t2 = toc;

ret2{1}{1}.time % show time log of the first vehicle from the first initial condition

speedUp = (1-t2/t1)*100