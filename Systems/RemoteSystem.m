
classdef RemoteSystem < DtSystem & InitDeinitObject
%% RemoteSystem discre-time remote system through UDP link
%
%   RemoteSystem(par1, val1, par2, val2, ... )
%
%   where the parameters are chosen among
%
%   'RemoteIp', 'RemotePort', 'LocalIp', 'LocalPort', 'MaxWaitingTime', 
%   'StateFeedback', 'StartingCommand'
%
%   Establish a UDP link from the local interface ('LocalIp':'LocalPort')
%   to a remote interface  ('RemoteIp':'RemotePort') in the remote system.
%   
%   This object implements a DtSystem and at every iteration of the simulation
%   sends the input vector u(k) to the remote system and receives the output 
%   vector y(k+1) (or the state vector x(k+1) if 'StateFeedback' is set to 1)
%
%   If the state vector is not received within 'MaxWaitingTime' seconds the
%   udp socket is closed.
%
%   At the beginning of the symulation the command 'StartingCommand' is
%   run, which can be used to start the remote robot.
%
%   VirtualArena sends the message 'CLOSE' to the remote system to close
%   it.
                            
                            
                        
%usefull commands :
%
%kill background process:
%ps -ax | grep Robot
%kill [id]
    
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
            
            %% Retrive parameters for superclass GeneralSystem
            
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