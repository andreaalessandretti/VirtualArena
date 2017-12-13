
classdef TestFminconMpcOpSolverSparse < matlab.unittest.TestCase
    
    methods (Test)
        
        function test_ComputeCost(testCase)
            
            [mpcOp,solver,U,x0,x1,x2,u0,u1,k0] = TestFminconMpcOpSolverSparse.getSolverTest();
          
            costDes = mpcOp.stageCost(k0,x0,u0) + mpcOp.stageCost(k0+1,x1,u1) + mpcOp.terminalCost(k0+2,x2);
            cost    = solver.fminconCost(mpcOp,k0,x0,U,{});
            
            testCase.verifyEqual(costDes,cost);
            
        end
        
        function test_ComputeEqualityConstraints(testCase)
            [mpcOp,solver,U,x0,x1,x2,u0,u1,k0] = TestFminconMpcOpSolverSparse.getSolverTest();
            
            f = @(k,x,u)mpcOp.system.f(k,x,u);
            
            CDes = [x1-f(k0  ,x0,u0);
                    x2-f(k0+1,x1,u1)];
                
            C    = solver.getNonlinearConstraintsCeq(mpcOp,k0,x0,U,{});
            
            testCase.verifyEqual(CDes,C);
            
        end
        
        function test_getNonlinearInequalityConstraints(testCase)
            
            [mpcOp,solver,U,x0,x1,x2,u0,u1,k0] = TestFminconMpcOpSolverSparse.getSolverTest();
            
            stageCon    = mpcOp.stageConstraints{1};% on the variable z=[x;u];
            terminalCon =  mpcOp.terminalConstraints{1};% on the variable z=[k;x];
            
            
            CDes = [stageCon.f([x0;u0]);stageCon.f([x1;u1]);terminalCon.f([k0+2;x2])];
            C = solver.getNonlinearConstraintsC(mpcOp,k0,x0,U,{});
            testCase.verifyEqual(CDes,C);
            
        end
        
    end
    
    
    methods (Static)
        function  [mpcOp,solver,U,x0,x1,x2,u0,u1,k0] = getSolverTest()
            
            stageCon    =  BoxSet(randn(4,1),randn(4,1)); % on the variable z=[x;u];
            terminalCon =  BoxSet(randn(4,1),randn(4,1)); % on the variable z=[k;x];
            
            mpcOp = IDtMpcOp( ...
                'System'               , IDtSystem('StateEquation', @(k,x,u) [sin(x(3));cos(x(3));u],'nx',3,'nu',1),...
                'HorizonLength'        , 2,...
                'StageConstraints'     , stageCon, ... 
                'TerminalConstraints'  , terminalCon, ... % on the variable z=[k,x];
                'StageCost'            , @(k,x,u,varargin)x'*x + 2*u'*u + k^2,...
                'TerminalCost'         , @(k,x,varargin) 3*x'*x + k^2 ...
                );
           
            
            solver = FminconMpcOpSolverSparse('MpcOp', mpcOp);
            
            x0 = randn(3,1);
            x1 = randn(3,1);
            x2 = randn(3,1);
            u0 = randn(1);
            u1 = randn(1);
            k0 = floor(10*randn);
            
            U = [x1;x2;u0;u1];
            
        end
        
    
    end
    
end
