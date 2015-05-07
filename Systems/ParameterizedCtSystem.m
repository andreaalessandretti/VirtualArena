%% ParametriedCtSystem, Abstract class for parameter estimation
% 
% Ex. parametric class
%
% classdef ParUnicycle < CtSystem & ParameterizedCtSystem  
%     methods
%         function obj =  ParUnicycle(par,varargin)
%             
%             obj = obj@CtSystem('nx',3,'nu',2,'ny',3,varargin{:});
%             obj.parameters = par;
%             
%             obj.f = @(t,x,u) obj.parF(t,x,u,obj.params);
%             obj.h = @(t,x)   x(1:3);
%             
%         end
%         
%         function dx = parametricF(obj,t,x,u,p)
%             
%             dx  =[cos(x(3))*p(1)*u(1);
%                   sin(x(3))*p(1)*u(1);
%                   u(2)*p(2) ];
%             
%         end      
%     end     
% end
%
% Ex. of use 
%
% load sim.mat
% 
% y = ret{1}.stateTrajectory';
% u = ret{1}.inputTrajectory';
% 
% modelSys = ParUnicycle([3;3],'InitialCondition',ones(3,1));
% 
% modelSys.parameters
% 
% modelSys.optimizeParameters(u,y,dt);
% 
% modelSys.parameters
%
classdef ParameterizedCtSystem < handle
    
    properties
        parameters;
    end
    
    methods (Abstract)
        parametricF
    end
    
    methods

        
        function [dx, y] = idnlgrayFnc(obj,t,x,u,p,varargin)
          % Output equations.
          y  = obj.h(t,x);
          dx = obj.parametricF(t,x,u,p);
        end
        
        function  optimizeParameters(obj,u,y,dt,varargin)
            
            display         = 0;
            fixedInitialCon = 0;
            
            %% Process options
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                    
                    switch varargin{parameterPointer}
                        
                        case 'Disply'
                            
                            display = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'FixedInitialCondition'
                            
                            fixedInitialCon = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                            
                    end
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            
            z = iddata(y, u, dt);
            
         
            FncName       = @ParameterizedCtSystem.nlgreyestFnc;       % File describing the model structure.
            Order         = [obj.ny obj.nu obj.nx];           % Model orders [ny nu nx].
            Parameters    = {obj.parameters};       % Initial parameters. Np = 2.
            InitialStates = obj.initialConditions;            % Initial initial states.
            Ts            = 0;                 % Time-continuous system.
           
            nlgr = idnlgrey(FncName, Order, Parameters, InitialStates, Ts);
            nlgr.FileArgument = {obj};
        
            InitialModel = nlgr;
            
            varInit = cell(1,obj.nx);
            [varInit{:}] = deal(fixedInitialCon);
            
            nlgr = setinit(nlgr, 'Fixed', varInit); % Estimate the initial states.
            opt = nlgreyestOptions('Display', 'on');
            nlgr = nlgreyest(z, nlgr, opt);

            nlgr.Report
            fprintf('\n\nThe search termination condition:\n')
            nlgr.Report.Termination
            OptimizedModel = nlgr;
            
            if display
                disp('Displaying comparison')
            	compare(z, OptimizedModel,InitialModel);
            end
            
            obj.parameters = nlgr.Parameters.Value;
        end
        
    end
    
    methods (Static)
        
    function [dx, y] = nlgreyestFnc(t, x, u, p, varargin)
        modelSys = varargin{1}{1};
        [dx, y] = modelSys.idnlgrayFnc(t,x,u,p);
    end
    
    end
    
end