
classdef TestLinearizedSystem < matlab.unittest.TestCase
    
    methods (Test)
        
        function testLin(testCase)
            
            tolleranceLin = 0.00001;
            sys  = MyCtSystem();
            
            sys.useSymbolicLinearization();
            
            %xDot = [k*x(1) + u;
            %        x(2)*u];
                
             A = @(k,x,u) [k,0; 0,u];
             B = @(k,x,u) [1; x(2)];
             C = @(k,x,u) eye(2);
             D = @(k,x,u) [0;0];
             
             p = @(kk,k,x,u) sys.f(k,x,u) +[x(1);0]*kk-[[x(1);0],A(k,x,u),B(k,x,u)]*[k;x;u];
             q = @(kk,k,x,u)[0;0];
             
             k  = randn(1);
             kk = randn(1);
             x  = randn(sys.nx,1);
             u  = randn(sys.nu,1);
             
             %% A
             expA = sys.A(k,x,u);
             actA = A(k,x,u);
             testCase.verifyEqual(expA,actA);
             
             sys.useLinearizationBySamples();
             
             expA = norm(sys.A(k,x,u)-actA)<tolleranceLin;
             testCase.verifyEqual(expA,true);
             
             sys.useSymbolicLinearization();
             
             %% B
             expB = sys.B(k,x,u);
             actB = B(k,x,u);
             testCase.verifyEqual(expB,actB);
             
             sys.useLinearizationBySamples();
             
             expB = norm(sys.B(k,x,u)-actB)<tolleranceLin;
             testCase.verifyEqual(expB,true);
             
             sys.useSymbolicLinearization();
             
             
             %% C
             expC = feval(sys.nameFunC,k,x,u);
             actC = C(k,x,u);
             testCase.verifyEqual(expC,actC);
             
             sys.useLinearizationBySamples();
             
             expC = norm(sys.C(k,x,u)-actC)<tolleranceLin;
             testCase.verifyEqual(expC,true);
             
             sys.useSymbolicLinearization();
             
             %D
             expD = sys.D(k,x,u);
             actD = D(k,x,u);
             testCase.verifyEqual(expD,actD);
             
             sys.useLinearizationBySamples();
             
             expD = norm(sys.D(k,x,u)-actD)<tolleranceLin;
             testCase.verifyEqual(expD,true);
             
             sys.useSymbolicLinearization();
             
             %p
             expp = sys.p(kk,k,x,u);
             actp = p(kk,k,x,u);
             testCase.verifyEqual(expp,actp);
             
             sys.useLinearizationBySamples();
             
             expp = norm(sys.p(kk,k,x,u)-actp)<tolleranceLin;
             testCase.verifyEqual(expp,true);
             
             sys.useSymbolicLinearization();
             
             %q
             expq = sys.q(kk,k,x,u);
             actq = q(kk,k,x,u);
             testCase.verifyEqual(expq,actq);
             
             sys.useLinearizationBySamples();
             
             expp = norm(sys.q(kk,k,x,u)-actq)<tolleranceLin;
             testCase.verifyEqual(expp,true);
        end
        
        
    end
    
end 
