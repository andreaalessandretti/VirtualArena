
classdef VirtualArena < handle
    %VirtualArena simulation environment
    %
    % va   = VirtualArena(sysList,par1,val1,par2,val2,...)
    % logs = va.run();
    %
    % Parameters:
    %
    %  'StoppingCriteria' :stop = @(t,sysList)
    %   When this function returns true, the simulation terminates.
    %       t       : simulation time
    %       sysList : the list of the systems
    %
    %   e.g.
    %       'StoppingCriteria' ,@(t,sysList) t >10
    %       'StoppingCriteria' ,@(t,sysList) norm(sysList{1}.x) <0.01
    %
    %
    %  'DiscretizationStep' : dt (double)
    %   Fixed length discretization step for continuous time simulation
    %
    %  'StepPlotFunction' : newHandles = @(sysList,log,oldHandles,i);
    %   Function called every 'PlottingStep' taking as inputs:
    %       sysList    : the list of the systems
    %       log        : the logs of the current simulation
    %       oldHandles : a variable returned in the previous call of the
    %                    'StepPlotFunction' usually containing plot handles
    %
    %  'InitPlotFunction' : @() (function with no parameters)
    %   Function called before the StepPlotFunction function is called for
    %   the first time
    %
    %  'StopPlotFunction' : @(logSim,vaObj)
    %   Function called at the end of the simulation that takes as input:
    %       logSim : the simulation logs (i.e., returned by va.run())
    %       vaObj  : a reference to the VirtualArena object.
    %
    %  'HandlePostFirstPlot' : @() (function with no parameters)
    %   Function called after the StepPlotFunction function is called for
    %   the first time
    %
    %  'PlottingStep' : dt
    %   VirtualArena calls the 'StepPlotFunction' when mod(t,dt)==0
    %
    %  'VideoName' : videoName (Default : [])
    %   If not empty, VA records a video of the simulation under videoName.avi
    %
    %
    %  'SensorsNetwork' : {s1,A1,s2,A2,...} (Default : {})
    %   with si belonging to the class Sensor() and Ai being the associated
    %   Adjacency matrix. 'SensorsNetwork' Defines the communication between
    %   systems, specifically each agent receives measurements from the
    %   sensor si from the neighborhood  defined by the Adjacency matrix
    %   Ai
    %
    %  'Integrator' : Integrator (Default : RK4())
    %   Object of the class Integrator defining the kind of integration
    %   method used in the simulation
    %
    %   e.g.
    %   'Integrator', EulerForward()
    %   'Integrator', RK4()
    %
    %  'ExtraLogs' : {log1, log2, ...}, with logi belonging to class Log
    %   You can add extra log to the classic ones (e.g., states, inputs, ...)
    %   providing new object of the class Log. See helo Log
    %
    %   e.g.
    %   'ExtraLogs', {Log('vLyap', @(t,sys,u) sys.controller.getVLyap(t,agent.x)) }
    %
    %   'MultiRuns' {m1, m2, ...} where mi is a MultiRun object
    %    run multiple simulations with different initializations.
    %    see help MultiRun
    %
    %   'RealTime' : 0 or 1. If 1 then the simulation time will go no
    %                faster than the real time.
    %
    %   'DefaultPlots' : 1 (default value) to use the default plots when
    %                    0 otherwise
    %
    % Example:
    %
    % va = VirtualArena({v1,v2},...
    %     'StoppingCriteria'  ,@(t,as)t>50,...
    %     'StepPlotFunction'  ,@(systemsList,log,oldHandles,k) someStepPlotFunction(systemsList,log,oldHandles,k,extraPar1,extraPar2), ...
    %     'StopPlotFunction'  ,@(allLogs,va)someStopPlotFunction(allLogs,va,extraPar3,extraPar4),...
    %     'SensorsNetwork'    , {s1,A},...
    %     'DiscretizationStep',dt,...
    %     'PlottingStep' ,1);
    %
    %   ret = va.run();
    %
    % See also handle
    
    % This file is part of VirtualArena.
    %
    % Copyright (c) 2014, Andrea Alessandretti
    % All rights reserved.
    %
    % e-mail: andrea.alessandretti [at] {epfl.ch, ist.utl.pt}
    %
    % Redistribution and use in source and binary forms, with or without
    % modification, are permitted provided that the following conditions are met:
    %
    % 1. Redistributions of source code must retain the above copyright notice, this
    %    list of conditions and the following disclaimer.
    % 2. Redistributions in binary form must reproduce the above copyright notice,
    %    this list of conditions and the following disclaimer in the documentation
    %    and/or other materials provided with the distribution.
    %
    % THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    % ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    % WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    % DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
    % ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    % (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    % LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    % ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    % (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    % SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    %
    % The views and conclusions contained in the software and documentation are those
    % of the authors and should not be interpreted as representing official policies,
    % either expressed or implied, of the FreeBSD Project.
    properties
        
        systemsList
        
        %stoppingCriteria is an handle function that defines the scenario when the
        %	simulation stops as function of the time/step number t and the system list.
        %
        %	e.g. Stop after 100 steps:
        %       @(k,systemsList) k>100; -
        %
        %   e.g. Stop when vehicle 1 reaches the origin
        %       @(k,systemsList) systemsList{1}.x(1:2)=[0;0];
        %
        stoppingCriteria  = @(t,sysList)t>10
        stoppingForced    = 0;
        %stepPlotFunction is the function handle executed at every step of the
        %   simulation
        %
        %	newHandles  = @(systemsList,log,oldHandles,k);
        %
        %   oldHandles is a vectors with the plot handles of the plots executed at
        %   previous step and newHandles contains the new plot handles.
        %
        %   e.g.
        % function h = stepPlotMass(systemsList,log,oldHandles,k)
        %
        %     if not(oldHandles == 0)
        %       delete(oldHandles)
        %     end
        %
        %     x = log{1}.stateTrajectory(:,1:k);
        %     h = plot(x(1,:),x(2,:));
        %
        % end
        stepPlotFunction
        
        %initPlotFunction is a function handle without parameters executed at the
        %   beginning of the simulation
        initPlotFunction
        
        %stopPlotFunction is the function handle executed at the end of the simulation
        %
        %	@(logAlls)
        %
        %	if simulations from different initial conditions are executed,
        %   logAlls{i} contains the log associated with the ith initial condition,
        %   otherwise logAlls = log
        stopPlotFunction
        
        stateTrajectory;
        
        computationTimes
        
        inputTrajectory;
        
        videoName = [];
        
        discretizationStep = 1;
        
        %sensorsNetwork = {s1,L1,s2,L2,...};
        %
        %   sk, with k = 1,2,..., belongs to the class Sensor
        %   if Lk(i,j) = 1 if system i senses system j with sensor sk, 0 otherwise.
        sensorsNetwork ={};
        
        %log of the simulation
        %
        %   log{i}.stateTrajectory(:,k) contains the state vector of system i at time k.
        %
        %   log{i}.inputTrajectory(:,k) contains the input vector applied
        %   to system i at time k.
        %
        %   log{i}.controllerStateTrajectory contains the state vector of
        %   the dynamic controller (if any) of system i at time k.
        log;
        
        logAll;
        
        plottingStep = [];
        
        %integrator is an oject of the class Integrator used to discretize
        %   continuous time systems (Default = RK4)
        integrator = RK4();
        
        logObjs
        
        multiruns = {};
        
        realTime = 0;
        
        defaultPlotMaxColumn = 5;
        
        defaultPlotMaxRows   = 6;
        
        defaultPlots         = 1;
        
        profiler             = 0;
        
        % DisplaySelector object that chooses what to display
        display              = 0;
        
        preSimuationTestFnc
        
        preSimuationTest = 0;
        
        initialTime = 0;
        
    end
    
    properties (SetAccess = private, GetAccess = private)
        
        blockSizeAllocation = 100;
        
        handlePostFirstPlot = [];
        
    end
    
    methods
        
        
        function obj = VirtualArena(varargin)
            %VirtualArena is the constructor
            %
            %   See help VirtualArena
            
            % Default Settings
            
            obj.logObjs = {InputLog(),StateLog(),ControllerStateLog(),ObserverStateLog(),TimeLog()};
            obj.display = DisplayNothing();
            obj.setOptions(varargin{:});
            
            if isempty(obj.plottingStep)
                obj.plottingStep = obj.discretizationStep;
            end
            
            if isempty(obj.multiruns)
                obj.multiruns = { MultiRun(1,@(i,va)1==1) }; % Just do the classic simulation without modifications
            end
            
        end
        
        %%callFunctionOnSystemsOnList(functionName,targetClass)
        %   execute the function .functionName on all the systems,
        %   controllers, and state observers that belong to the class
        %   targetClass
        function callFunctionOnSystemsOnList(obj,functionName,targetClass)
            
            for i = 1:length(obj.systemsList)
                
                if isa(obj.systemsList{i},targetClass)
                    obj.systemsList{i}.(functionName);
                end
                
            end
            
        end
        
        
        function logAll = run(obj)
            
            obj.callFunctionOnSystemsOnList('initSimulations','InitDeinitObject')
            
            if isa(obj.initPlotFunction,'function_handle')
                obj.initPlotFunction();hold on
            elseif obj.monodimentionalSystems()
                obj.oneDinitPlotFunction();hold on
            end
            
            logAll = obj.recursiveRun(obj.multiruns);
            
            obj.logAll = logAll;
            obj.callFunctionOnSystemsOnList('deinitSimulations','InitDeinitObject')
            
        end
        
        
        function log = runOne(obj,varargin)
            
            if nargin == 2 %Simulate using initial condition iInitialCondition
                
                iInitialCondition = varargin{1};
                
            else %Single initial conditions
                
                iInitialCondition = 0;
                
            end
            
            obj.callFunctionOnSystemsOnList('initSimulation','InitDeinitObject')
            
            obj.loadInitialConditions(iInitialCondition);
            
            i = 1;
            
            if obj.preSimuationTest
                retTest = obj.preSimuationTestFnc(obj);
                if not(isempty(retTest))
                    log = retTest;
                    return
                end
            end
            
            plot_handles = 0;
            
            if ischar(obj.videoName)
                %aviobj = avifile(strcat(obj.videoName,'.avi'),'compression','None');
                aviobj = VideoWriter(strcat(obj.videoName,'.avi'));
                open(aviobj);
                
            end
            
            %% Compute time for obj.initLogs(timeInfo);
            if not(obj.discretizationStep==1) %TODO: remove this madness
                timeInfo = obj.initialTime + (i-1)*obj.discretizationStep;
            else
                timeInfo = obj.initialTime + (i-1);
            end
            
            obj.initLogs(timeInfo);
            
            if not(obj.discretizationStep==1) && obj.realTime
                simTimeTic =tic;
            end
            
            if obj.profiler
                profile on
            end
            
            
            while not( obj.stoppingCriteria(timeInfo,obj.systemsList)  || obj.stoppingForced )
                
                
                
                
                %% Compute time
                if not(obj.discretizationStep==1) %TODO: remove this madness
                    timeInfo = obj.initialTime + (i-1)*obj.discretizationStep;
                else
                    timeInfo = obj.initialTime + (i-1);
                end
                
                for ia = 1:length(obj.systemsList) % Main loop for a single system
                    
                    z = []; %measurament
                    
                    %% Update current state of the systems
                    if not(isempty(obj.systemsList{ia}.stateObserver)) % Observer feedback
                        
                        xObs          = obj.systemsList{ia}.stateObserver.x;
                        xToController = obj.systemsList{ia}.stateObserver.h(timeInfo,xObs);
                        x             = obj.systemsList{ia}.x;
                        
                        %elseif not(isempty(obj.systemsList{ia}.h)) % Output feedback
                        %
                        %    x             = obj.systemsList{ia}.x;
                        %    xToController = obj.systemsList{ia}.h(timeInfo,x);
                        %    z             = xToController;
                        
                    else % State feedback
                        
                        x             = obj.systemsList{ia}.x;
                        xToController = x;
                        
                    end
                    z = obj.systemsList{ia}.h(timeInfo,x);
                    %% Classic vs Network control
                    if not(isempty(obj.sensorsNetwork))
                        
                        netReadings = obj.senseNetworkToAgent(timeInfo,ia);
                        
                        if isempty(netReadings)
                            netReadings = {};
                        end
                        
                    else
                        netReadings = {};
                    end
                    
                    controllerFParams = {xToController,netReadings{:}};
                    %netReadings{1}{i_sensor}{j_measurament_of_sensor_i}
                    
                    %% Compute input
                    uSysCon = [];
                    if isa(obj.systemsList{ia}.controller,'CtSystem') %CtController
                        
                        %xc = obj.log{ia}.controllerStateTrajectory(:,i);
                        xc = obj.systemsList{ia}.controller.x;
                        
                        uSysCon = obj.systemsList{ia}.controller.h(timeInfo,xc,controllerFParams{:});
                        
                        nextXc = obj.integrator.integrate( @(xc)obj.systemsList{ia}.controller.f(timeInfo,xc,uSysCon,controllerFParams{:}),xc,obj.discretizationStep);
                        
                        nextXc = obj.systemsList{ia}.controller.updateState(timeInfo,nextXc,uSysCon,controllerFParams{:});
                        
                        obj.systemsList{ia}.controller.x = nextXc;
                        
                        u = uSysCon(1:obj.systemsList{ia}.nu);
                        
                    elseif isa(obj.systemsList{ia}.controller,'DtSystem') %DtController
                        
                        xc = obj.systemsList{ia}.controller.x;
                        
                        uSysCon = obj.systemsList{ia}.controller.h(timeInfo,xc,controllerFParams{:});
                        
                        nextXc = obj.systemsList{ia}.controller.f(timeInfo,xc,uSysCon,controllerFParams{:});
                        
                        nextXc = obj.systemsList{ia}.controller.updateState(timeInfo,nextXc,uSysCon,controllerFParams{:});
                        
                        obj.systemsList{ia}.controller.x = nextXc;
                        
                        u = uSysCon(1:obj.systemsList{ia}.nu);
                        
                    elseif isa(obj.systemsList{ia}.controller,'Controller') %Memoryless Controller
                        
                        %% TO DO: to fix this exception
                        if isa(obj.systemsList{ia}.controller,'ControllerAdapter')
                            u = obj.systemsList{ia}.controller.computeInput(timeInfo,controllerFParams{1});
                        else
                            
                            nInputController = nargin(sprintf('%s>%s.%s',class(obj.systemsList{ia}.controller),class(obj.systemsList{ia}.controller),'computeInput'))-1;
                            
                            if nInputController==2 %does not consider network readings
                                u = obj.systemsList{ia}.controller.computeInput(timeInfo,controllerFParams{1});
                            else
                                u = obj.systemsList{ia}.controller.computeInput(timeInfo,controllerFParams{:});
                            end
                        end
                        
                    elseif isempty(obj.systemsList{ia}.controller) && (isempty(obj.systemsList{ia}.nu) || obj.systemsList{ia}.nu == 0 ) %Automnomus System
                        
                        u=[];
                        
                    else
                        
                        error(getMessage('VirtualArena:UnknownControllerType'));
                        
                    end
                    
                    %% Cv vs Dt System
                    parameterF = {u};
                    
                    if isa(obj.systemsList{ia},'CtSystem')
                        
                        if isempty(obj.discretizationStep)
                            
                            error(getMessage('VirtualArena:discretizationStepNotNefined'));
                        end
                        
                        nextX = obj.integrator.integrate( @(x)obj.systemsList{ia}.f(timeInfo,x,parameterF{:}),x,obj.discretizationStep);
                        
                        nextX = obj.systemsList{ia}.updateState(timeInfo,nextX,parameterF{:});
                        
                    elseif isa(obj.systemsList{ia},'DtSystem')
                        
                        nextX = obj.systemsList{ia}.f(timeInfo,x,parameterF{:});
                        nextX = obj.systemsList{ia}.updateState(timeInfo,nextX,parameterF{:});
                        
                    else
                        error(getMessage('VirtualArena:UnknownSystemType'));
                    end
                    
                    if not(obj.discretizationStep==1)&& obj.realTime
                        while toc(simTimeTic)<timeInfo*obj.realTime
                        end
                    end
                    
                    if isa(obj.systemsList{ia}.stateObserver,'DynamicalSystem')
                        
                        [z,xObsNext] = obj.manageObserver(timeInfo,netReadings,ia,x,u,nextX);
                        
                    end
                    
                    obj.appendLogs(obj.systemsList{ia},u,ia,i,timeInfo,z,netReadings,uSysCon);
                    
                    nextXs{ia} = nextX;
                    
                    if isa(obj.systemsList{ia}.stateObserver,'DynamicalSystem')
                        
                        xObsNexts{ia} = xObsNext;
                    end
                    
                end
                
                %% Update state objects
                
                for ia = 1:length(obj.systemsList) % Main loop for a single system
                    obj.systemsList{ia}.x = nextXs{ia};
                    
                    if isa(obj.systemsList{ia}.stateObserver,'DynamicalSystem')
                        
                        obj.systemsList{ia}.stateObserver.x = xObsNexts{ia};
                    end
                end
                %% Plots
                if isa(obj.stepPlotFunction,'function_handle') && mod(timeInfo,obj.plottingStep)==0
                    
                    plot_handles  = obj.stepPlotFunction(obj.systemsList,obj.log,plot_handles,i);
                    
                    if isa(obj.handlePostFirstPlot,'function_handle') && i==1
                        obj.handlePostFirstPlot();
                    end
                    
                    drawnow
                    if ischar(obj.videoName)
                        F = getframe(gcf);
                        %aviobj = addframe(aviobj,F);
                        writeVideo(aviobj,F)
                    end
                    
                elseif   mod(timeInfo,obj.plottingStep)==0 && obj.defaultPlots
                    
                    
                    plot_handles  = obj.defaultStepPlotFunction(obj.systemsList,obj.log,plot_handles,i);
                    
                    if isa(obj.handlePostFirstPlot,'function_handle')&&i==1
                        
                        obj.handlePostFirstPlot();
                        
                    elseif i==1
                        
                        obj.defaultAfterStepPlotFunction();
                    end
                    
                    
                    drawnow
                    if ischar(obj.videoName)
                        F = getframe(gcf);
                        %aviobj = addframe(aviobj,F);
                        %aviobj = addframe(aviobj,F);
                        writeVideo(aviobj,F)
                    end
                end
                
                i = i+1;
                
                drawnow
                if strcmp(get(gcf,'CurrentCharacter'),'q')==1
                    obj.stoppingForced = 1;
                    disp('PRESSED KEY ''q'' >> simulation stopped')
                end
                
            end
            if obj.profiler
                profile viewer
            end
            obj.callFunctionOnSystemsOnList('deinitSimulation','InitDeinitObject');
            
            if ischar(obj.videoName)
                
                aviobj = close(aviobj);
                
            end
            
            obj.cutExtraLogVector(i-1,timeInfo);
            log = obj.log;
        end
        
        function cutLogVector(obj,fildname,i)
            for iAgent = 1:length(obj.systemsList)
                if not(i>=size(obj.log{iAgent}.(fildname),2))
                    newLog = obj.log{iAgent}.(fildname);
                    
                    obj.log{iAgent}.(fildname) =  newLog(:,1:i);
                    
                end
            end
            
        end
        
        function cutExtraLogVector(obj,i,t)
            
            for iAgent = 1:length(obj.systemsList)
                
                logObjs = obj.logObjs;
                
                for j = 1:length(logObjs)
                    
                    if isempty(logObjs{j}.condition) || (not(isempty(logObjs{j}.condition)) && logObjs{j}.condition(t,obj.systemsList{iAgent}))
                        
                        fildname = logObjs{j}.name;
                        iS = i + logObjs{j}.shift;
                        if not(iS>=size(obj.log{iAgent}.(fildname),2))
                            newLog = obj.log{iAgent}.(fildname);
                            
                            obj.log{iAgent}.(fildname) =  newLog(:,1:iS);
                            
                        end
                    end
                    
                end
                
            end
            
        end
        
        function appendVectorToLog(obj,v,iAgent,fildname,i)
            
            if i>=size(obj.log{iAgent}.(fildname),2) % Allocate memory
                
                obj.log{iAgent}.(fildname) =  [obj.log{iAgent}.(fildname),zeros(size(v,1),obj.blockSizeAllocation)];
                
            end
            obj.log{iAgent}.(fildname)(:,i) = v;
            
        end
        
        function appendLogs(obj,agent,u,iAgent,i,t,z,netReadings,uSysCon)
            
            logObjs = obj.logObjs;
            
            for j = 1:length(logObjs)
                
                logger = logObjs{j};
                
                iLog       = i;
                logTheDate = 1;
                
                if ~isempty(logger.deltaStep)
                    logTheDate = mod(i,logger.deltaStep)==0;
                    iLog = i/logger.deltaStep +1; % plus one is because 0 is considered
                end
                
                if ~isempty(logger.deltaTime)
                    logTheDate = mod(t,logger.deltaTime)==0;
                    iLog = t/logger.deltaTime +1; % plus one is because 0 is considered
                end
                
                
                if logTheDate && (isempty(logger.condition) || (not(isempty(logger.condition)) && logger.condition(t,agent,u,z)))
                    obj.appendVectorToLog(logger.getVectorToLog(t,agent,u,z,netReadings,uSysCon)    ,iAgent,logger.name,iLog + logger.shift );
                    %logger.i=logger.i+1;
                end
                
            end
            
        end
        
        function initLogs(obj,t0)
            
            
            nSystems = length(obj.systemsList);
            
            %% Initialize simulation
            for i = 1:nSystems
                
                logObjs = obj.logObjs;
                if not(isempty(logObjs))
                    for j = 1:length(logObjs)
                        logObjs{j}.i=1;
                        if isempty(logObjs{j}.condition) || ( not(isempty(logObjs{j}.condition)) && logObjs{j}.condition(t0,obj.systemsList{i}))
                            if not(isempty(logObjs{j}.initialization))
                                obj.log{i}.(logObjs{j}.name) = logObjs{j}.initialization;
                            else
                                obj.log{i}.(logObjs{j}.name) = logObjs{j}.getVectorToLog(t0,obj.systemsList{i});
                            end
                        end
                        
                    end
                    
                end
                
                
            end
        end
        
        
        function oneDinitPlotFunction(obj)
        end
        
        function h = plotSignalPredictionAndEstimate(obj,signal,estimate,prediction,timeSignal,timePrediction)
            
            h = [];
            
            maxColumn = obj.defaultPlotMaxColumn;
            maxRows   = obj.defaultPlotMaxRows;
            
            nv = size(signal,1);
            
            nColumns = ceil(nv/maxRows);
            if nColumns> maxColumn
                return
            end
            nRows    = ceil(nv/nColumns);
            
            plotEstimate   = ~isempty(estimate);
            plotPrediction = ~isempty(prediction);
            
            h = zeros((plotEstimate+plotPrediction+1)*nv,1);
            iPlot = 1;
            
            for i=1:nv
                
                subplot(nRows,nColumns,i);
                
                h(iPlot) = plot(timeSignal,signal(i,:)); hold on
                iPlot = iPlot+1;
                
                if plotPrediction
                    h(iPlot) = plot(timePrediction,prediction(i,:),'--'); hold on
                    iPlot = iPlot+1;
                end
                
            end
            
            for i=1:size(estimate,1)
                if plotEstimate
                    subplot(nRows,nColumns,i);
                    h(iPlot) = plot(timeSignal,estimate(i,:),'--'); hold on
                    iPlot = iPlot+1;
                end
            end
            
        end
        
        function h = defaultStepPlotFunction(obj,systemsList,log,oldHandles,k,varargin)
            if oldHandles~=0
                delete(oldHandles);
            end
            
            if nargin ==6
                figO = varargin{1}-1;
            else
                figO = 0;
            end
            
            h = [];
            
            for sysId = 1:length(systemsList)
                figure(figO+sysId);
                est   = [];
                pred  = [];
                tPred = [];
                t = log{sysId}.time(:,1:k);
                x = log{sysId}.stateTrajectory(:,1:k);
                u = log{sysId}.inputTrajectory(:,1:k);
                
                if ~isempty(systemsList{sysId}.stateObserver)
                    est = log{sysId}.observerStateTrajectory(1:systemsList{sysId}.nx,1:k);
                end
                
                figure(figO+sysId);
                hi = obj.plotSignalPredictionAndEstimate([x;u],est,pred,t,tPred);
                h = [h;hi];
                
            end
            
        end
        
        function  defaultAfterStepPlotFunction(obj,varargin)
            systemsList = obj.systemsList;
            if nargin ==2
                figO = varargin{1}-1;
            else
                figO = 0;
            end
            
            for sysId = 1:length(systemsList)
                
                figure(figO+sysId)
                xlabel = {};
                ulabel = {};
                nx = systemsList{sysId}.nx;
                nu = systemsList{sysId}.nu;
                
                if iscell(systemsList{sysId}.stateName) && length(systemsList{sysId}.stateName)==nx
                    xlabel = systemsList{sysId}.stateName;
                else
                    for ix = 1:nx
                        xlabel{ix}=sprintf('x%i',ix);
                    end
                end
                
                if iscell(systemsList{sysId}.inputName) && length(systemsList{sysId}.inputName)==nu
                    ulabel = systemsList{sysId}.stateName;
                else
                    for iu = 1:nu
                        ulabel{iu}=sprintf('u%i',iu);
                    end
                end
                labels = {xlabel{:},ulabel{:}};
                
                
                maxColumn = obj.defaultPlotMaxColumn;
                maxRows   = obj.defaultPlotMaxRows;
                
                nv = nx+nu;
                
                nColumns = ceil(nv/maxRows);
                if nColumns> maxColumn
                    return
                end
                nRows    = ceil(nv/nColumns);
                
                for i = 1:nv
                    subplot(nRows,nColumns,i);
                    ylabel(labels{i});
                    grid on;
                    setNicePlot
                end
                
            end
            
        end
        
        function h = oneDStepPlotFunction(obj,systemsList,log,oldHandles,k)
            
            if iscell(oldHandles)
                
                delete(oldHandles{1})
                delete(oldHandles{2})
                
            end
            
            nAgents = length(systemsList);
            
            hup   = zeros(1,nAgents);
            hdown = zeros(1,nAgents);
            
            indexPlotsUp = 1;
            indexPlotsDown = 1;
            
            for i= nAgents
                
                x = log{i}.stateTrajectory(:,1:k);
                u = log{i}.inputTrajectory(:,1:k);
                
                t = log{i}.time(:,1:k);
                
                subplot(2,1,1);
                
                hup(indexPlotsUp) = plot(t,x); hold on
                
                indexPlotsUp = indexPlotsUp+1;
                
                
                subplot(2,1,2);
                
                hdown(indexPlotsDown) = plot(t,u);  hold on
                
                indexPlotsDown = indexPlotsDown+1;
                
                if isa(systemsList{i}.controller,'MpcController')
                    x_opt  = systemsList{i}.controller.lastSolution.x_opt;
                    u_opt  = systemsList{i}.controller.lastSolution.u_opt;
                    tu_opt = systemsList{i}.controller.lastSolution.tu_opt;
                    tx_opt = systemsList{i}.controller.lastSolution.tx_opt;
                    
                    subplot(2,1,1);
                    hup(indexPlotsUp) = plot(tx_opt,x_opt,'--');hold on;
                    indexPlotsUp=indexPlotsUp+1;
                    
                    subplot(2,1,2);
                    hdown(indexPlotsDown) = plot(tu_opt,u_opt,'--');hold on;
                    indexPlotsDown=indexPlotsDown+1;
                    
                end
            end
            h={hup,hdown};
            
            
        end
        
        %%monodimentionalSystems returns one if the size of all the
        %% systems is one, zero otherwise
        function ret =  monodimentionalSystems(obj)
            ret = 1;
            for i = 1:length(obj.systemsList)
                if not(obj.systemsList{i}.nx ==1)
                    ret = 0;
                    return;
                end
            end
            
        end
        
        function oneDPostFirstPlot(obj)
            
            subplot(2,1,1);
            title('State')
            xlabel('t [sec]')
            ylabel('x(t)')
            grid on
            setNicePlot
            
            subplot(2,1,2);
            title('Input')
            xlabel('t [sec]')
            ylabel('u(t)')
            setNicePlot
            
        end
        
        
        
    end
    
    
end