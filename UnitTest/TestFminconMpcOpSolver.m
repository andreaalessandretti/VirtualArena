
classdef TestFminconMpcOpSolver < matlab.unittest.TestCase
    
    methods (Test)
        
        function test_UseSymbolicEvaluation(testCase)
             import matlab.unittest.constraints.IsEqualTo;
       import matlab.unittest.constraints.AbsoluteTolerance;
       import matlab.unittest.TestCase;
            v = 2;
            
            beta  = 0;
            A=[1,1;0,1];
            A1 = eye(2);
            A2 = A;
            A3 = -1;
            B  = [0;1];
            alpha = [0,1.7,0];
            
            % A1 D^{0} x + A2 D^{0.7} x= B1 u
            % D^{0.7} x = -A2^{-1}*A1 D^{0} x +A2^{-1}* B1 u
            
            dtSys  = DtFractionalOrderSystem(alpha,beta,{A1,A2,A3},{B});
            model = TruncatedDtFractionalOrderSystem(dtSys,v);
            
            e = @(k,x) x(1:2);
            
            mpcOp = IDtMpcOp( ...
                'System'               , model,...
                'HorizonLength'        , 5,...
                'StageConstraints'     , BoxSet( -1,model.nx+1,1,model.nx+1,model.nx+1),... % on the variable z=[x;u];
                'StageCost'            , @(k,x,u,varargin) 100*x'*x +10* e(k,x)'* e(k,x) ...
                );
            
            solver1 = FminconMpcOpSolver('MpcOp', mpcOp,'UseSymbolicEvaluation',1);
            solver1.initSimulation();
            
            solver2 = FminconMpcOpSolver('MpcOp', mpcOp);
            
            nU =solver1.getSizeOptimizer;
            U = rand(nU,1);
            netReadings = {};
            k0 = 1;
            x0 = rand(model.nx,1);
            
            cost1 = solver1.fminconCostSym(k0,x0,U,netReadings)';
            
            cost2 = solver2.fminconCost(mpcOp,k0,x0,U,netReadings)';
            
            cons1 =  solver1.fminconConSym2(k0,x0,U,netReadings)';
            
            cons2 =  solver2.getNonlinearConstraints(mpcOp,k0,x0,U,netReadings)';
            
            %testCase.verifyEqual(cons1,cons2);
            
            %testCase.verifyEqual(cost1,cost2);
            testCase.assertThat(cons1, IsEqualTo(cons2, 'Within', AbsoluteTolerance(exp(-4))));
            testCase.assertThat(cost1, IsEqualTo(cost2, 'Within', AbsoluteTolerance(exp(-4))));

        end
    end
    
end
