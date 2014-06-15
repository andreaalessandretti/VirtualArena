

classdef ConsensusController < CtSystem
    %Vehicle
    
    properties
        lastReadings;
        Agraph;          % Vector containing the weight for formation keeping control
        communicationDt;
        d;
        trackingLaw;
        
    end
    
    methods
        
        
        function obj = ConsensusController(d, Agraph, communicationDt,trackingLaw)
            
            if trackingLaw.vehicle.n ==2
                nx=2;nu=2;ny=2;
            else
                nx=3;nu=12;ny=2;
            end
            
            obj = obj@CtSystem('nx',nx,'nu',nu,'ny',ny);
            
            obj.trackingLaw  = trackingLaw;
            obj.f = @(x,u,t,readings)obj.xcDot(x,readings);
            obj.h = @(x,z,t,readings)obj.computeInput(x,z,readings);
            obj.d = d;
            obj.Agraph = Agraph;
            obj.communicationDt = communicationDt;
            
        end
        
        
        function xcDot = xcDot(obj,xc,readings)
           
            xcDot = ConsensusController.computeUConsensus(xc,readings,obj.lastReadings,obj.communicationDt,obj.Agraph,obj.d,obj.trackingLaw.vehicle.n);
           
        end
        
        function u = computeInput(obj,xc,x,readings)
           
             u2 = ConsensusController.computeUConsensus(xc,readings,obj.lastReadings,obj.communicationDt,obj.Agraph,obj.d,obj.trackingLaw.vehicle.n);
             
             obj.trackingLaw.pd    = @(t)xc;
             obj.trackingLaw.dotPd = @(t)u2; 
             u = obj.trackingLaw.computeInput(x,0);
             
             %delta = [1 0 -obj.e(3) obj.e(2); 0 obj.e(3) 0 -obj.e(1); 0 -obj.e(2) obj.e(1) 0];
             %deltaBar = delta'/(delta*delta');
             %R = reshape(x(4:12),3,3);
             %err = R'*(x(1:3)-xc)-obj.e;
             %u = deltaBar*(R'*u2-err);
             
             
             obj.lastReadings = readings;
              
        end
    end
    
    methods (Static)
        function u2 = computeUConsensus(xc,readings,lastReadings,communicationDt,Agraph,d,nSpace)
            
            
            n_detect = nnz(Agraph);
            nnz_A = find(Agraph);
            eta = sum(Agraph);
            gamma = 1;
            % computeInput of the simple integrator part of
            % extandedUnicycle
            
            u2 = zeros(nSpace,1);
            
            for i=1:n_detect
                
                if isempty(lastReadings)
                    vi = zeros(size( readings{1}{i}));
                else
                    vi = (readings{1}{i}-lastReadings{1}{i})/communicationDt;
                end
                
                dx = readings{1}{i}-xc;
                
                u2 = u2 + 1/eta*Agraph(nnz_A(i))*(vi+gamma*(dx+d(:, nnz_A(i))) );
            end
             
            
        end
        
        
    end
    
end