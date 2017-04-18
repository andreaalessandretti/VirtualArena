
classdef TestWarmStarts < matlab.unittest.TestCase
    
    methods (Test)
        
        function test_ShiftAndAppendAuxLawWarmStart(testCase)
            
            sol.x_opt = [0,1,2,3];
            sol.u_opt = [4,5,6];
            
            a = MyShiftAndAppendAuxLawWarmStart();
            
            actSolution = a.generateWarmStarts(0,sol);
            
            expSolution.x_opt = [1,2,3,0];
            expSolution.u_opt = [5,6,-3];
            
            testCase.verifyEqual(actSolution,expSolution);
        end
        
        function test_IShiftAndAppendAuxLawWarmStart(testCase)
            
            sol.x_opt = [0,1,2,3];
            sol.u_opt = [4,5,6];
            
            a = IShiftAndAppendAuxLawWarmStart(@(t,x)-x*exp(-t),@(t,x,u)x*exp(-t)+u);
            
            actSolution = a.generateWarmStarts(0,sol);
            
            expSolution.x_opt = [1,2,3,0];
            expSolution.u_opt = [5,6,-3];
            
            testCase.verifyEqual(actSolution,expSolution);
        end
        
        function test_ShiftAndAppendZeroWarmStart(testCase)
            
            sol.x_opt = [0,1,2,3];
            sol.u_opt = [4,5,6];
            
            a = ShiftAndAppendZeroWarmStart();
            
            actSolution = a.generateWarmStarts(0,sol);
            
            expSolution.x_opt = [1,2,3,0];
            expSolution.u_opt = [5,6,0];
            
            testCase.verifyEqual(actSolution,expSolution);
        end
        
        function test_ShiftAndHoldWarmStart(testCase)
            
            sol.x_opt = [0,1,2,3];
            sol.u_opt = [4,5,6];
            
            a = ShiftAndHoldWarmStart();
            
            actSolution = a.generateWarmStarts(0,sol);
            
            expSolution.x_opt = [1,2,3,3];
            expSolution.u_opt = [5,6,6];
            
            testCase.verifyEqual(actSolution,expSolution);
        end
        
        function test_ZerosWarmStart(testCase)
            
            sol.x_opt = [0,1,2,3];
            sol.u_opt = [4,5,6];
            
            a = ZerosWarmStart();
            
            actSolution = a.generateWarmStarts(0,sol);
            
            expSolution.x_opt = [0,0,0,0];
            expSolution.u_opt = [0,0,0];
            
            testCase.verifyEqual(actSolution,expSolution);
        end
        
        function test_IAuxLawWarmStart(testCase)
            
            sol.x_opt  = [rand(1,1),1,rand(1,1)];
            sol.u_opt  = [randn(1,2)];
            sol.tx_opt = [1,2,3];
            a = IAuxLawWarmStart(@(k,x)k*x,@(k,x,u)k*x+u,1);
            
            actSolution = a.generateWarmStarts(rand(1),sol);
            
            expSolution.x_opt  = [1,4,24];
            expSolution.u_opt  = [2,12];
            expSolution.tx_opt = [2,3,4];
            testCase.verifyEqual(actSolution,expSolution);
        end
        
        function test_MyAuxLawWarmStart(testCase)
            
            sol.x_opt  = [rand(1,1),1,rand(1,1)];
            sol.u_opt  = [randn(1,2)];
            sol.tx_opt = [1,2,3];
            
            a = MyAuxLawWarmStart();
            
            actSolution = a.generateWarmStarts(rand(1),sol);
            
            expSolution.x_opt  = [1,4,24];
            expSolution.u_opt  = [2,12];
            expSolution.tx_opt = [2,3,4];
            testCase.verifyEqual(actSolution,expSolution);
        end
        
    end
    
end 
