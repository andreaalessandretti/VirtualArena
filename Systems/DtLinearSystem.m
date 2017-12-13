
%% Linear Systems
%
classdef DtLinearSystem < DtSystem
    
    
    properties
        A
        B
        G
        
        C
        D
        
        addNoise = 0;
    end
    
    methods
        
        function obj = DtLinearSystem (varargin)
            
            obj = obj@DtSystem(varargin{:});
            
            
            %% Retrive parameters for superclass DynamicalSystem
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'A'
                            
                            obj.A = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                        
                        case 'B'
                        
                            obj.B = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                         case 'G'
                            
                            obj.G = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                        case 'C'
                            
                            obj.C = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                        
                        case 'D'
                        
                            obj.D = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                      
                        case 'AddNoise'
                        
                            obj.addNoise = 1;
                            parameterPointer = parameterPointer+1;
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
            end
            
            if isempty(obj.A) || not(size(obj.A,1)==size(obj.A,2))
               	error('(Square) matrix ''A'' required.');
            end
            
            obj.nx = size(obj.A,1);
            
            if isempty(obj.C)
                obj.C = eye(obj.nx);
            end
            
            obj.ny = size(obj.C,1);
            
            if isempty(obj.B)
                obj.nu = 0;
            else
                obj.nu = size(obj.B,2);
            end
            
            if isempty(obj.D)
                obj.D = zeros(size(obj.C,1),obj.nx);
            end
        end
        
        function xNext = f(obj,t,x,u,varargin)
            
            xNext = obj.A*x+obj.B*u;
            
             if obj.addNoise == 1 && not(isempty(obj.G))
             xNext = xNext + obj.G*randn(size(obj.G,2),1);
             elseif obj.addNoise == 2 && not(isempty(obj.G))
             xNext = xNext + obj.G*(rand(size(obj.G,2),1)-0.5);
             end
            
        end
        
        function y = h(obj,t,x,varargin)
            
            y = obj.C*x;
            
        end
        
        function ret = computeMaximumRCIS(obj,G,bSet)

            sys=ULTISystem('A',obj.A,'B',obj.B,'E',G);
            sys.x.min = bSet.lowerBounds(1:obj.nx);
            sys.x.max = bSet.upperBounds(1:obj.nx);
            sys.u.min = bSet.lowerBounds(obj.nx+(1:obj.nu));
            sys.u.max = bSet.upperBounds(obj.nx+(1:obj.nu));
            sys.d.min = bSet.lowerBounds(obj.nx+obj.nu+(1:size(G,2)));
            sys.d.max = bSet.upperBounds(obj.nx+obj.nu+(1:size(G,2)));
            
            ret = sys.invariantSet();
            
        end
        
        function ret = computeMaximumRCIS_a(obj,K,G,bSet)

            sys=ULTISystem('A',obj.A+obj.B*K,'E',G);
            sys.x.min = bSet.lowerBounds(1:obj.nx);
            sys.x.max = bSet.upperBounds(1:obj.nx);
            sys.d.min = bSet.lowerBounds(obj.nx+(1:size(G,2)));
            sys.d.max = bSet.upperBounds(obj.nx+(1:size(G,2)));
            
            ret = sys.invariantSet();
            
        end
        
        
    end
end