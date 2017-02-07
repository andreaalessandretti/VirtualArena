%% Observer managment
% 1) postInputUpdate
%    update using the computed input (for the case of input dependent output)
% 2) predict
% 3) preInputUpdate
%
% The updates are executed only if the output is
% not empty

function [z,xObsNext] = manageObserver(obj,timeInfo,netReadings,ia,x,u,nextX)

z = [];
xObs          = obj.systemsList{ia}.stateObserver.x;

measureAndFilter = mod(round(timeInfo/obj.discretizationStep),obj.systemsList{ia}.stateObserver.downsampling)==0;

if measureAndFilter
    %% Compute measuraments
    zSys = [];
    if  ~isempty(obj.systemsList{ia}.h)
        if nargin(obj.systemsList{ia}.h)==2
            zSys = obj.systemsList{ia}.h(timeInfo,x);
        elseif nargin(obj.systemsList{ia}.h)==3
           
            zSys = obj.systemsList{ia}.h(timeInfo,x,u);
        end
    end
    zNet = [];
    if ~isempty(netReadings)
        for si=1:length(netReadings)
            zNet = [zNet;cell2mat(netReadings{si})];
        end
    end
    
    z=[zSys;zNet];
    %% Update (using input)
    if(isa(obj.systemsList{ia}.stateObserver,'StateObserver') && ~isempty(z))
        xObs = obj.systemsList{ia}.stateObserver.postInputUpdate(timeInfo,xObs,z,u);
    end
end

% Prediction
if isa(obj.systemsList{ia}.stateObserver,'CtSystem')
    xObsNext = obj.integrator.integrate( @(xObs)obj.systemsList{ia}.stateObserver.f(timeInfo,xObs,[u;z]),xObs,obj.discretizationStep);
elseif isa(obj.systemsList{ia}.stateObserver,'DtSystem')
    xObsNext = obj.systemsList{ia}.stateObserver.f(timeInfo,xObs,[u;z]);
end



if isa(obj.systemsList{ia},'CtSystem')
    timeInfoNext = timeInfo+obj.discretizationStep;
elseif isa(obj.systemsList{ia},'DtSystem')
    timeInfoNext = timeInfo+1;
end

measureAndFilterNext = mod(round(timeInfoNext/obj.discretizationStep),obj.systemsList{ia}.stateObserver.downsampling)==0;
if measureAndFilterNext
    % Update of the predicted state (without using the input)
    if(isa(obj.systemsList{ia}.stateObserver,'StateObserver')) &&  nargin(obj.systemsList{ia}.h)==2
        
        zNext    = obj.systemsList{ia}.h(timeInfoNext,nextX);
        if ~isempty(zNext)
            xObsNext = obj.systemsList{ia}.stateObserver.preInputUpdate(timeInfoNext,xObsNext,zNext);
        end
    end
end
end