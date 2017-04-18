
classdef TestSymbolizedSystem < matlab.unittest.TestCase
    
    methods (Test)
        
        
        function test_SDtSystem(testCase)
            
            sys  = MyDtSystem();
            
            ssys = SDtSystem(sys);
            
            TestSymbolizedSystem.add_testSystem(testCase,sys,ssys)
            
        end
        
        function test_SCtSystem(testCase)
            
            sys  = MyCtSystem();
            
            ssys = SCtSystem(sys);
            
            TestSymbolizedSystem.add_testSystem(testCase,sys,ssys)
            
        end
    end
    
    methods(Static)
        
        function add_testSystem(testCase,sys,ssys)
            
            t = randn(1);
            x = randn(sys.nx,1);
            u = randn(sys.nu,1);
            
            actXdot = sys.f(t,x,u);
            expXdot = ssys.f(t,x,u);
            
            actY = sys.h(t,x,u);
            expY = ssys.h(t,x,u);
            
            expPar = sys.getParameters();
            actPar = ssys.getParameters();
            
            testCase.verifyEqual(actXdot,expXdot);
            testCase.verifyEqual(actY,expY);
            testCase.verifyEqual(actPar,expPar);
            
        end
        
    end
    
end
