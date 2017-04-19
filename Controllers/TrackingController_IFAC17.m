classdef TrackingController_IFAC17 < Controller
    
    properties
        epsilon
        cd
        cdDot
        K
        PinvE
    end
    
    methods
        
        function obj = TrackingController_IFAC17(varargin)
            
            obj = obj@Controller();
            
            
            %% Retrive parameters for superclass GeneralSystem
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                     
                        case 'Epsilon'
                            
                            obj.epsilon = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'cd'
                            
                            obj.cd = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'cdDot'
                            
                            obj.cdDot = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'K'
                            
                            obj.K = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
                
                
            end
           
            if isempty(obj.epsilon)
                error(getMessage('UtilsMsg:missingParameter','Epsilon'));
            end
            %if isempty(obj.cd)
            %    error(getMessage('UtilsMsg:missingParameter','cd'));
            %end
            %if isempty(obj.cdDot)
            %    error(getMessage('UtilsMsg:missingParameter','cdDot'));
            %end
            if isempty(obj.K)
                error(getMessage('UtilsMsg:missingParameter','Ke'));
            end
          
  
            OmegaEpsilon = @(epsilon)[-epsilon(2);epsilon(1)];
            
            epsilon = obj.epsilon;
            
            Lvk = [1;0];
            E   = [Lvk,OmegaEpsilon(epsilon)];
            
            if not(rank(E)==size(E,1))
                error('System not controllable, try a different epsilon');
            end
            
            obj.PinvE = inv(E);
            
        end
        
        
        function u = computeInput(obj,t,x)
            
            u = obj.computeInputFromCdCdDot(obj.cd(t),obj.cdDot(t),x);
            
        end
        
        function u = computeInputFromCdCdDot(obj,cd,cdDot,x)
            
            p = x(1:2);
            
            R = [cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
            
            PinvE = obj.PinvE;
            
            y = R'*(p-cd)+obj.epsilon;
            
            r= -R'*cdDot ;
            
            u = -PinvE*r;
            if norm(y)>0.01
                u = u - PinvE*obj.K*y/sqrt(y'*y);
            end
        end
        
        
        function y = computeOutput(obj,t,x)
            
            y = obj.computeOutputFromCd(obj.cd(t),x);
            
        end
        
        function y = computeOutputFromCd(obj,cd,x)
            
            R = [cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];
            
            p = x(1:2);
            
            y = R'*(p-cd)+obj.epsilon;
            
        end
        
        function ret = getMaxInput(obj,maxNormCdot)

            ret       = [0;0];
            DeltaInv  = obj.PinvE;
            DeltaInvK = obj.PinvE*obj.K;
            
            ret(1) = norm(DeltaInv(1,:))*maxNormCdot+norm(DeltaInvK(1,:));
            ret(2) = norm(DeltaInv(2,:))*maxNormCdot+norm(DeltaInvK(2,:));

        end
        
    end
    
end