classdef TruncatedDtFractionalOrderSystem < DtLinearSystem
    
    properties
        v
        originalSystem
        checkA % checkA_n is in checkA{n+1}
        checkB % checkB_n is in checkB{n+1}
        hatA % hatA_n is in hatA{n+1}
        hatB % hatB_n is in hatB{n+1}
        assumptionParams
        tildeGv
    end
    
    methods
        function obj = TruncatedDtFractionalOrderSystem(dtFos,v)
            
            %% Note: j in [0,v] is the paper is jj in [1,v+1]
            
            for jj = 1:v+1 %0:v
                hatA{jj}=zeros(size(dtFos.As{1}));
                for ii = 1:length(dtFos.alpha)
                    hatA{jj}=hatA{jj}+dtFos.As{ii}*FractionalOrderSystem.c_alpha_j(dtFos.alpha(ii),jj-1);
                end
                
                hatB{jj}=zeros(size(dtFos.Bs{1}));
                for ii = 1:length(dtFos.beta)
                    hatB{jj}=hatB{jj}+dtFos.Bs{ii}*FractionalOrderSystem.c_alpha_j(dtFos.beta(ii),jj-1);
                end
            end
            
            for jj = 1:v+1 %(0:v)
                checkA{jj} = -inv(hatA{1})*hatA{jj};
                checkB{jj} =  inv(hatA{1})*hatB{jj};
            end
            
            tildeA = zeros(v*(dtFos.nx+dtFos.nu));
            
            for ii = 1:v
                
                tildeA(1:dtFos.nx, (ii-1)*dtFos.nx + (1:dtFos.nx) ) = checkA{ii+1};
                
                tildeA(1:dtFos.nx, dtFos.nx*v + (ii-1)*dtFos.nu + (1:dtFos.nu) ) = checkB{ii+1};
                
            end
            
            lengthIx = dtFos.nx*(v-1);
            lengthIu = dtFos.nu*(v-1);
            
            tildeA(dtFos.nx + (1:lengthIx), 1:lengthIx) = eye(lengthIx);
            
            tildeA(dtFos.nx + lengthIx + dtFos.nu + (1:lengthIu), lengthIx + dtFos.nx + (1:lengthIu)) = eye(lengthIu);
            
            tildeB = zeros( size(tildeA,1),dtFos.nu);
            
            tildeB(1:dtFos.nx,:) = checkB{1};
            
            tildeB(dtFos.nx+lengthIx+(1:dtFos.nu),:) = eye(dtFos.nu);
            
            %if dtFos.beta == 0
            %    A = A(1:dtFos.nx+lengthIx,1:dtFos.nx+lengthIx);
            %    B = B(1:dtFos.nx+lengthIx,:);
            %end
            
            tildeG = zeros(size(tildeA,1),dtFos.nx);
            if  length(dtFos.Gs)==1 && dtFos.gamma ==0
                tildeG(1:dtFos.nx,:) = dtFos.Gs{1};
            elseif length(dtFos.Gs)>0
                disp('G not supported');
            end
            
            obj = obj@DtLinearSystem('A',tildeA,'B',tildeB,'G',tildeG);
            
            obj.tildeGv = zeros(size(tildeA,1),dtFos.nx);
            obj.tildeGv(1:dtFos.nx,:) = eye(dtFos.nx);
            
            obj.originalSystem = dtFos;
            obj.v = v;
            
            obj.checkA = checkA;
            obj.checkB = checkB;
            obj.hatA = hatA;
            obj.hatB = hatB;
            
        end
%         
%         function ret = getSigmaBar(obj,v,A_K,G,P,Q)
%             
%             ret = 0;
%             alphaBar = TruncatedDtFractionalOrderSystem.getAlphaBar(A_K,G,P,Q);
%             alphas = obj.originalSystem.alpha;
%             betas = obj.originalSystem.beta;
%             As = obj.originalSystem.As;
%             Bs  = obj.originalSystem.Bs;
%             for ii = 1:length(alphas)
%                 sigma_i = TruncatedDtFractionalOrderSystem.getSigma(v+1,alphas(ii),A_K,G,P,Q);
%                 ret = ret + norm(inv(obj.hatA{1})*As{ii})*alphaBar*sigma_i;
%             end
%             
%             for ii = 1:length(betas)
%                 sigma_i = TruncatedDtFractionalOrderSystem.getSigma(v+1,betas(ii),A_K,G,P,Q);
%                 ret = ret + norm(inv(obj.hatA{1})*Bs{ii})*sigma_i;
%             end
%             
%         end
%         
%         
        function ret = compressStateFromTo(obj,x,v1,v2)
            
            nxx = obj.originalSystem.nx;
            nuu = obj.originalSystem.nu;
            selector = [(1:nxx*v2), nxx*v1+(1:nuu*v2)];
            ret = x(selector);
            
        end
%         
%         function ret = getXfos(obj,UXoriginal)
%             xMin = UXoriginal.lowerBounds(1:obj.originalSystem.nx);
%             uMin = UXoriginal.lowerBounds(obj.originalSystem.nx+(1:obj.originalSystem.nu));
%             xMax = UXoriginal.upperBounds(1:obj.originalSystem.nx);
%             uMax = UXoriginal.upperBounds(obj.originalSystem.nx+(1:obj.originalSystem.nu));
%             ret = BoxSet([repmat(xMin,obj.v,1);repmat(uMin,obj.v,1)],[repmat(xMax,obj.v,1);repmat(uMax,obj.v,1)]);
%         end
%         
%         function ret = getXfos_beta0(obj,UXoriginal)
%             xMin = UXoriginal.lowerBounds(1:obj.originalSystem.nx);
%             xMax = UXoriginal.upperBounds(1:obj.originalSystem.nx);
%             ret = BoxSet(repmat(xMin,obj.v,1),repmat(xMax,obj.v,1));
%         end
%         
%         
%         function ret = beta_ss(obj,A_K,G,P,Q,K)
%             
%             alpha = obj.originalSystem.alpha;
%             beta  = obj.originalSystem.beta;
%             
%             ret = 0;
%             
%             for i=1:length(alpha)
%                 %ret = ret + |hatA0^{-1}*Ai|* barAlpha * beta_s(alpha_i/barAlpha,v)
%                 Ai = obj.originalSystem.As{i};
%                 
%                 hatA0 = obj.hatA{1};
%                 alphaBar = TruncatedDtFractionalOrderSystem.getAlphaBar(A_K,G,P,Q);
%                 beta_s = TruncatedDtFractionalOrderSystem.beta_s(alpha(i)/alphaBar,obj.v+1);
%                 ret = ret + norm( inv(hatA0)*Ai)*alphaBar*beta_s;
%             end
%             
%             for i=1:length(beta)
%                 %ret = ret + |hatA0^{-1}*Bi*K| * beta_s(beta_i/barAlpha,v)
%                 Bi = obj.originalSystem.Bs{i};
%                 hatA0 = obj.hatA{1};
%                 alphaBar = TruncatedDtFractionalOrderSystem.getAlphaBar(A_K,G,P,Q);
%                 beta_s = TruncatedDtFractionalOrderSystem.beta_s(beta(i)/alphaBar,obj.v+1);
%                 ret = ret + norm( inv(hatA0)*Bi*K)*beta_s;
%             end
%             
%             
%         end
%         
%         function assSatisfied = satisfiesAssTh1(obj,Qlqr,Rlqr,varargin)
%             
%             if nargin >5
%                 hatTheta = varargin{1};
%                 theta    = varargin{2};
%                 crho     = varargin{3};
%             else
%                 hatTheta =0.9;
%                 theta = 0.5;
%                 crho =0.9;
%             end
%             
%             
%             [K,P,E] = dlqr(obj.A,obj.B,Qlqr,Rlqr);
%             A_K = obj.A-obj.B*K;
%             
%             if max(real(eig(A_K)).*real(eig(A_K)))>1
%                 error('1');
%             end
%             
%             Gv = obj.G;
%             
%             lmabda_min_Q = min(real(eigs(Qlqr)));
%             lmabda_min_P = min(real(eigs(P)));
%             lmabda_max_P = min(real(eigs(P)));
%             
%             lmabda_max_GPG = min(real(eigs(Gv'*P*Gv)));
%             
%             c4 = (1-theta)*lmabda_min_Q/lmabda_max_P;
%             
%             hatC4 = min(c4,hatTheta);
%             
%             c2 = lmabda_max_GPG + norm(Gv'*P*A_K)^2/(theta*lmabda_min_Q);
%             c_gamma = sqrt(c2/(hatC4*crho*lmabda_min_P));
%             
%             assSatisfied = c_gamma*obj.Psi(K)<1;
%             obj.assumptionParams.K = K;
%             
%         end
%         
%         function assSatisfied = satisfiesAssTh1Old(obj,Qlqr,Rlqr,theta)
%             
%             [K,P,E] = dlqr(obj.A,obj.B,Qlqr,Rlqr);
%             A_K = obj.A-obj.B*K;
%             
%             if max(real(eig(A_K)).*real(eig(A_K)))>1
%                 error('1');
%             end
%             
%             Gv = obj.G;
%             
%             lmabda_min_Q = min(real(eigs(Qlqr)));
%             
%             c_ub  = (1-theta)*lmabda_min_Q;
%             
%             c2 = max(real(eigs(Gv'*P*Gv))) - norm(Gv'*P*A_K)^2/(theta*lmabda_min_Q);
%             
%             
%             assSatisfied = c2*obj.beta_ss(A_K,Gv,P,Qlqr,K)^2 < c_ub;
%             
%             obj.assumptionParams.K = K;
%             
%         end
        
         function ret = phi(obj,alpha)
          
            ret = exp(alpha);
            
            if obj.v>=0
                for j=0:obj.v
                    ret = ret-((alpha)^j/factorial(j));
                end
            end
            ret = max(ret,0);
            
         end
         
         function ret = Psi(obj,K)
            
            alpha = obj.originalSystem.alpha;
            beta  = obj.originalSystem.beta;
            
            ret = 0;
            invHatA0 = inv(obj.hatA{1});
            
            for i=1:length(alpha)
                Ai = obj.originalSystem.As{i};
                ret = ret + norm(invHatA0*Ai)*obj.phi(alpha(i));
            end
            
            for i=1:length(beta)
                Bi = obj.originalSystem.Bs{i};
                ret = ret + norm(invHatA0*Bi*K)*obj.phi(beta(i));
            end
            
         end
         
        
    end
    methods (Static)
        
%         
%         function ret = beta_s(alpha,v)
%             
%             ret = exp(alpha);
%             for j=0:v-1
%                 ret = ret-((alpha)^j/factorial(j));
%             end
%             
%         end
%         
%         
%         function ret = getSigma(v,alpha,A_K,G,P,Q)
%             
%             alphaBar = TruncatedDtFractionalOrderSystem.getAlphaBar(A_K,G,P,Q);
%             
%             ret = exp(alpha/alphaBar);
%             
%             for j=0:v-1
%                 ret = ret-(alpha/alphaBar)^j/factorial(j);
%             end
%             
%         end
%         
%         
%         
%         function alphaBar = getAlphaBar(A_K,G,P,Q)
%             
%             Pmin = min(eigs(P));
%             Pmax = max(eigs(P));
%             Qmax = max(eigs(Q));
%             b1   = norm(G'*P*A_K)/sqrt(min(eigs(G'*P*G)));
%             alphaBar = sqrt(  ( 1- (b1^2+Qmax)/Pmin )*(Pmin/Pmax));
%             
%         end
%         
        
    end
    
end