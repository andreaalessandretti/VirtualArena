
classdef VirtualArena < handle
    %VirtualArena simulated environment
    %
    % VirtualArena Properties:
    %
    % stoppingCriteria   - stopping criteria of the simulation
    % systemsList        - cell containing the systems in the environment to simulate
    % log                - logs of the simulation
    % initPlotFunction   - plot executed at the beginning of the simulation
    % stepPlotFunction   - plot executed every 'plottingFrequency' steps of the simulation
    % stopPlotFunction   - plot executed at end of the simulation
    % plottingFrequency  - use 'stepPlotFunction' every 'plottingFrequency' steps
    % videoName          - record a video of the simulation under videoName.avi
    % discretizationStep - discretization step used to discretize continuous
    %                      time systems, if any
    % sensorsNetwork     - definition of the sensor network
    %
    % VirtualArena Methods:
    %
    % VirtualArena - costructor method
    % run          - start the simulation
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
        stoppingCriteria  = @(x,i) norm(x)<0.002
        
        %stepPlotFunction is the function handle executed at every step of the
        %   simulation
        %
        %	newHandles  = @(systemsList,log,oldHandles,k);
        %
        %   oldHandles is a vectors with the plot handles of the plots executed at
        %   previous step and newHandles contains the new plot handles.
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
        
        discretizationStep = [];
        
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
        
        plottingFrequency = 1;
        
        %integrator is an oject of the class Integrator used to discretize
        %   continuous time systems (Default = RK4)
        integrator = RK4();
        
    end
    
    properties (SetAccess = private, GetAccess = private)
        
        blockSizeAllocation = 100;
        
        handlePostFirstPlot = [];
        
    end
    
    methods
        
        
        function obj = VirtualArena(varargin)
            %VirtualArena is the costructor
            %
            %        va = VirtualArena(par1,val1,par2,val2,...)
            %
            %	where the parameters are chosen among the following
            %
            %	'StoppingCriteria'
            %	'StepPlotFunction'
            %	'InitPlotFunction'
            %	'StopPlotFunction'
            %	'HandlePostFirstPlot'
            %	'VideoName'
            %	'DiscretizationStep'
            %	'SensorsNetwork'
            %
            %	see the descriptions of the associated properties.
            %
            %   See also VirtualArena
            
            obj.setOptions(varargin{:});
            
            
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
            %run runs the simulation
            %
            %   log = va.run();
            %
            %   See also VirtualArena.log
            %
            
            obj.callFunctionOnSystemsOnList('initSimulations','InitDeinitObject')
             
            if isa(obj.initPlotFunction,'function_handle')
                obj.initPlotFunction();hold on
            end
            
            if isa(obj.systemsList{1}.initialConditions,'cell')
                
                nInitialConditoins = length(obj.systemsList{1}.initialConditions);
                logAll = cell(1,nInitialConditoins);
                
                for i = 1:nInitialConditoins
                    logAll{i} = obj.runOne(i);
                end
                
                if isa(obj.stopPlotFunction,'function_handle')
                    obj.stopPlotFunction(logAll,obj); hold on
                end
                
            else
                
                logAll = obj.runOne();
                
                if isa(obj.stopPlotFunction,'function_handle')
                    
                    obj.stopPlotFunction(logAll,obj); hold on
                end
                
            end
            
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
            
            obj.logInitialConditions(iInitialCondition);
            
            i = 2;
            
            plot_handles = 0;
            
            if ischar(obj.videoName)
                aviobj = avifile(strcat(obj.videoName,'.avi'),'compression','None');
            end
            
            %% Update current state of vehicles and controllers (initialization)
            for ia = 1:length(obj.systemsList)
                
                obj.systemsList{ia}.x=obj.log{ia}.stateTrajectory(:,i-1);
                
                if isa(obj.systemsList{ia}.controller,'GeneralSystem')
                    obj.systemsList{ia}.controller.x = obj.log{ia}.controllerStateTrajectory(:,i-1);
                end
                
            end
            
            
            while not( obj.stoppingCriteria(i,obj.systemsList) )
                
                %% Update current state of the systems 
                for ia = 1:length(obj.systemsList)
                    
                    obj.systemsList{ia}.x=obj.log{ia}.stateTrajectory(:,i-1);
                    
                    if isa(obj.systemsList{ia}.controller,'GeneralSystem')
                        obj.systemsList{ia}.controller.x = obj.log{ia}.controllerStateTrajectory(:,i-1);
                    end
                    
                end
                
                for ia = 1:length(obj.systemsList) % Main loop for a single system
                    
                    
                    %% Update current state of the systems
                    if not(isempty(obj.systemsList{ia}.stateObserver))
                        
                        xObs = obj.log{ia}.observerStateTrajectory(:,i-1);
                        xToController = obj.systemsList{ia}.stateObserver.h(xObs);
                        x = obj.log{ia}.stateTrajectory(:,i-1);
                        
                    else
                        
                        x = obj.log{ia}.stateTrajectory(:,i-1);
                        xToController = x;
                        
                    end
                    
                    
                    %% Classic vs Network control
                    if not(isempty(obj.sensorsNetwork))
                        
                        log = obj.senseNetworkToAgent(ia);
                        if isempty(log)
                            netReadings = {};
                        else
                            netReadings = obj.senseNetworkToAgent(ia);
                        end
                        
                    else
                        netReadings = {};
                    end
                    
                    %% Cv vs Dt vs Memoryless Controller
                    
                    
                    % Compute time
                    if isa(obj.systemsList{ia},'CtSystem')
                        timeInfo = i*obj.discretizationStep;
                    elseif isa(obj.systemsList{ia},'DtSystem')
                        timeInfo = i;
                    end
                    
                    
                    controllerFParams = {xToController,timeInfo,netReadings{:}};
                    
                    % Compute input
                    
                    if isa(obj.systemsList{ia}.controller,'CtSystem') %CtController
                        
                        xc = obj.log{ia}.controllerStateTrajectory(:,i-1);
                        
                        nextXc = obj.integrator.integrate( @(xc)obj.systemsList{ia}.controller.f(xc,controllerFParams{:}),xc,obj.discretizationStep);
                        
                        u = obj.systemsList{ia}.controller.h(xc,controllerFParams{:});
                        
                        obj.appendVectorToLog(nextXc,ia,'controllerStateTrajectory',i);
                        
                        
                    elseif isa(obj.systemsList{ia}.controller,'DtSystem') %DtController
                        
                        xc = obj.log{ia}.controllerStateTrajectory(:,i-1);
                        
                        nextXc = obj.systemsList{ia}.controller.f(xc,controllerFParams{:});
                        
                        u = obj.systemsList{ia}.controller.h(xc,controllerFParams{:});
                        
                        obj.appendVectorToLog(nextXc,ia,'controllerStateTrajectory',i);
                        
                        
                    elseif isa(obj.systemsList{ia}.controller,'Controller') %Memoryless Controller
                        
                        u = obj.systemsList{ia}.controller.computeInput(controllerFParams{:});
                        
                    else
                        
                        error(getMessage('VirtualArena:UnknownControllerType'));
                        
                    end
                    
                    
                    %% Cv vs Dt System
                    if isempty(obj.systemsList{ia}.Q)
                        parameterF = {u};
                    else
                        cQ = chol(obj.systemsList{ia}.Q)';
                        parameterF = {u,cQ*randn(length(cQ),1)};
                    end
                    
                    if isa(obj.systemsList{ia},'CtSystem')
                        
                        if isempty(obj.discretizationStep)
                            error(getMessage('VirtualArena:discretizationStepNotNefined'));
                        end
                        
                        nextX = obj.integrator.integrate( @(x)obj.systemsList{ia}.f(x,parameterF{:}),x,obj.discretizationStep);
                        
                    elseif isa(obj.systemsList{ia},'DtSystem')
                        
                        nextX = obj.systemsList{ia}.f(x,parameterF{:});
                        
                    else
                        error(getMessage('VirtualArena:UnknownSystemType'));
                    end
                    
                    if isa(obj.systemsList{ia}.stateObserver,'GeneralSystem')
                        
                        if isempty(obj.systemsList{ia}.R)
                            parameterH = {nextX,u};
                        else
                            cR = chol(obj.systemsList{ia}.R)';
                            parameterH = {nextX,u,cR*randn(length(cR),1)};
                        end
                        
                        z = obj.systemsList{ia}.h(parameterH{:});
                        
                    end
                    
                    %% Cv vs Dt Observer
                    if isa(obj.systemsList{ia}.stateObserver,'CtSystem')
                        
                        xObsNext = obj.integrator.integrate( @(xObs)obj.systemsList{ia}.stateObserver.f(xObs,[u;z]),xObs,obj.discretizationStep);
                        obj.appendVectorToLog(xObsNext,ia,'observerStateTrajectory',i);
                        
                    elseif isa(obj.systemsList{ia}.stateObserver,'DtSystem')
                        
                        xObsNext = obj.systemsList{ia}.stateObserver.f(xObs,[u;z]);
                        obj.appendVectorToLog(xObsNext,ia,'observerStateTrajectory',i);
                        
                    end
                    
                    obj.appendVectorToLog(nextX,ia,'stateTrajectory',i);
                    obj.appendVectorToLog(u    ,ia,'inputTrajectory',i-1);
                    
                end
                
                %% Plots
                if isa(obj.stepPlotFunction,'function_handle') && mod(i,obj.plottingFrequency)==0
                    
                    plot_handles  = obj.stepPlotFunction(obj.systemsList,obj.log,plot_handles,i);
                    
                    
                    if isa(obj.handlePostFirstPlot,'function_handle')&&i==2
                        obj.handlePostFirstPlot();
                    end
                    
                    drawnow
                    if ischar(obj.videoName)
                        F = getframe(gcf);
                        aviobj = addframe(aviobj,F);
                    end
                    
                end
                
                i = i+1;
                
            end
            
            obj.callFunctionOnSystemsOnList('deinitSimulation','InitDeinitObject');
            
            if ischar(obj.videoName)
                
                aviobj = close(aviobj);
                
            end
            
            
            obj.cutLogVector('stateTrajectory',i-1);
            obj.cutLogVector('inputTrajectory',i-2);
            
            log = obj.log;
        end
        
        function readings = senseNetworkToAgent(obj,systemId)
            
            nSensor = length(obj.sensorsNetwork)/2;
            emptyMeasurements = 1;
            
            readings = cell(nSensor,1);
            for i =1:nSensor
                
                sensor = obj.sensorsNetwork{(i-1)*2+1};
                Lsensor = obj.sensorsNetwork{(i-1)*2+2};
                
                detectableAgents  = Lsensor(systemId,:);
                indexesDetectable = 1:length(Lsensor(systemId,:));
                indexesDetectable = indexesDetectable(logical(detectableAgents));
                
                readings{i} = sensor.sense(systemId,obj.systemsList{systemId},obj.systemsList(indexesDetectable),indexesDetectable);
                
                if not(isempty(readings{i}))
                    emptyMeasurements = 0;
                end
                
            end
            
            if emptyMeasurements
                readings = {};
            else
                readings = {readings};
            end
            
        end
        
        
        
        
        function cutLogVector(obj,fildname,i)
            for iAgent = 1:length(obj.systemsList)
                if not(i>=size(obj.log{iAgent}.(fildname),2))
                    newLog = obj.log{iAgent}.(fildname);

                    obj.log{iAgent}.(fildname) =  newLog(:,1:i);

                end
            end
            
        end 
            
        function appendVectorToLog(obj,v,iAgent,fildname,i)
            
            if i>=size(obj.log{iAgent}.(fildname),2) % Allocate memory
                
                obj.log{iAgent}.(fildname) =  [obj.log{iAgent}.(fildname),zeros(size(v,1),obj.blockSizeAllocation)];
                
            end
            
            obj.log{iAgent}.(fildname)(:,i) = v;
            
        end
        
        
        function setOptions(obj,varargin)
            
            if isa(varargin{1},'cell')
                obj.systemsList = varargin{1};
            else
                obj.systemsList = {varargin{1}};
            end
            
            parameterPointer = 2;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        
                        case 'StoppingCriteria'
                            
                            obj.stoppingCriteria = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StepPlotFunction'
                            
                            obj.stepPlotFunction = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'InitPlotFunction'
                            
                            obj.initPlotFunction = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StopPlotFunction'
                            
                            obj.stopPlotFunction = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                            
                        case 'HandlePostFirstPlot'
                            
                            obj.handlePostFirstPlot =varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'VideoName'
                            
                            obj.videoName =varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'DiscretizationStep'
                            
                            obj.discretizationStep = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'SensorsNetwork'
                            
                            obj.sensorsNetwork = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'PlottingFrequency'
                            
                            obj.plottingFrequency = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'Integrator'
                            
                            obj.integrator = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
        end
        
        function logInitialConditions(obj,iInitialCondition)

            nSystems = length(obj.systemsList);
            
            obj.log = cell(1,nSystems);
            
            %% Initialize simulation
            for i = 1:nSystems
                
                
                if iInitialCondition
                    
                    % Log initial stateof the system
                    obj.log{i}.stateTrajectory = obj.systemsList{i}.initialConditions{iInitialCondition};
                    
                    % Log initial state of the observer
                    if not(isempty(obj.systemsList{i}.stateObserver))
                        obj.log{i}.observerStateTrajectory = obj.systemsList{i}.stateObserver.initialConditions{iInitialCondition};
                    end
                    
                    % Log initial state of the controller
                    if isa(obj.systemsList{i}.controller,'CtSystem') || isa(obj.systemsList{i}.controller,'DtSystem')
                        iCons = obj.systemsList{i}.controller.initialConditions;
                        
                        if not(iscell(iCons)) || length(iCons)<iInitialCondition
                            error(getMessage('VirtualArena:NotEnoughInitialConditionsController'));
                        end
                        
                        obj.log{i}.controllerStateTrajectory = obj.systemsList{i}.controller.initialConditions{iInitialCondition};
                    end
                    
                else
                    % Log the ith initial state of the system
                    obj.log{i}.stateTrajectory = obj.systemsList{i}.initialConditions;
                    
                    % Log initial state of the observer
                    if not(isempty(obj.systemsList{i}.stateObserver))
                        
                        if isempty(obj.systemsList{i}.stateObserver.initialConditions)
                            error(getMessage('VirtualArena:InitObserver'));
                        end
                        
                        obj.log{i}.observerStateTrajectory = obj.systemsList{i}.stateObserver.initialConditions;
                    end
                    
                    % Log initial state of the controller
                    if isa(obj.systemsList{i}.controller,'CtSystem') || isa(obj.systemsList{i}.controller,'DtSystem')
                        if isempty(obj.systemsList{i}.controller.initialConditions)
                            error(getMessage('VirtualArena:InitController'));
                        end
                        obj.log{i}.controllerStateTrajectory = obj.systemsList{i}.controller.initialConditions;
                    end
                end
                
                obj.log{i}.inputTrajectory = zeros(obj.systemsList{i}.nu,1);
                
            end
            
        end
        
        
    end
   
end