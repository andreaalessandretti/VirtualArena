
classdef RemoteSystem < DtSystem & InitDeinitObject
    
%usefull commands :
%
%kill background process:
%ps -ax | grep Robot
%kill [id]
    properties
        
        remoteIp
        remotePort
        localIp
        localPort
        udpObj
        maxWaitingTime = 10;
        stateFeedback = 0;
        log
        iLog = 1;
        startingCommand = [];
    end
    
    
    methods
        
        
        function obj = RemoteSystem (varargin)
            
            obj = obj@DtSystem(varargin{:});
            
            %% Retrive parameters for superclass DynamicalSystem
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'RemoteIp'
                            
                            obj.remoteIp = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'RemotePort'
                            
                            obj.remotePort = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'LocalIp'
                            
                            obj.localIp = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'LocalPort'
                            
                            obj.localPort = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'MaxWaitingTime'
                            
                            obj.maxWaitingTime = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StateFeedback'
                            
                            obj.stateFeedback = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'StartingCommand'
                            
                            obj.startingCommand = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            if isempty(obj.remoteIp) || isempty(obj.localIp) || isempty(obj.remotePort) || isempty(obj.localPort)
                error('The parameters ''RemoteIp'', ''LocalIp'', ''RemotePort'', and ''LocalPort'' are required.')
            end
            
            %% Ask initial Condition
            if isempty(obj.nx)
            	obj.nx=1;
            end
            
            obj.log.travelTime = [];
            
            obj.initialConditions = zeros(obj.nx,1);
            
           
            if obj.stateFeedback
                obj.f = @(t,x,u)obj.fRemote(x,u);
            else
                obj.f = @(t,x,u)zeros(obj.nx,1);
                obj.h = @(t,x,u)obj.fRemote(x,u);
                
            end
            
        end
        
        function xNext = fRemote(obj,x,u)
            srtime = tic;
            obj.send(u);
            xNext = obj.recive();
            
            obj.appendVectorToLog(toc(srtime),'travelTime',obj.iLog);
            obj.iLog = obj.iLog +1;
        end
        
        function closeConnection(obj)
            fclose(obj.udpObj);
            delete(obj.udpObj);
            disp('Connection Closed')
        end
        
        function send(obj,data)
            
            fwrite(obj.udpObj, data, 'double');
            
        end
        
        function data = recive(obj)
            
            try
            waitingTime = 0;
            
            tic
            
            while not(obj.udpObj.BytesAvailable) && waitingTime<=obj.maxWaitingTime
                
                waitingTime = toc;
                
            end
            
            if waitingTime>obj.maxWaitingTime
                fclose(obj.udpObj);
                delete(obj.udpObj);
                error(['Timeout (',num2str(obj.maxWaitingTime),')']);
            end
            
            data = fread(obj.udpObj, obj.udpObj.BytesAvailable,'double');
            
            catch e
                e
                obj.closeConnection();
            end
            
            
        end
        
        function init(obj)
            
            obj.udpObj = udp(obj.remoteIp, obj.remotePort, 'LocalPort', obj.localPort,'timeout',30,'OutputBufferSize',2048,'InputBufferSize',2048);
            fopen(obj.udpObj);
            
        end
        
        function appendVectorToLog(obj,v,fildname,i)
            
            if i>=size(obj.log.(fildname),2) % Allocate memory
                
                obj.log.(fildname) =  [obj.log.(fildname),zeros(size(v,1),100)];
                
            end
            
            obj.log.(fildname)(:,i) = v;
            
        end
        
        function initSimulation(obj)
            
            if obj.startingCommand
                system(obj.startingCommand);
            end
            pause(5);
            
            
            obj.udpObj = udp(obj.remoteIp, obj.remotePort, 'LocalPort', obj.localPort,'timeout',30,'OutputBufferSize',2048,'InputBufferSize',2048);
            fopen(obj.udpObj)
            disp('Connection open')
            
            if obj.stateFeedback

                fprintf('Request initial condition ...');
                obj.send(zeros(obj.nu,1));

                x0 = obj.recive();
                obj.initialConditions = x0;
                disp(' recived.');
                
            end
        end
        
        function deinitSimulation(obj)
            fwrite(obj.udpObj,'CLOSE','char');
            obj.closeConnection();
        end
        
    end
end