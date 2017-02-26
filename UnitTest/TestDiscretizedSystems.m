
classdef TestDiscretizedSystems < matlab.unittest.TestCase
    
    methods (Test)
        
        function test_DiscretizedSystem(testCase)
            
            sys = MyCtSystem();
            
            x = randn(sys.nx,1);
            u = randn(sys.nu,1);
            k = ceil(1+10*randn(1,1));
            
            dt = 0.1;
            
            dtSys = DiscretizedSystem(sys,dt);
            
            nextX   = @(k,x,u)RK4.integrate(@(y)sys.f(dt*k,y,u),x,dt);
            
            expX = dtSys.f(k,x,u);
            actX = nextX(k,x,u);
            
            testCase.verifyEqual(expX,actX);
            
            %%----
            
            dtSysEf = DiscretizedSystem(sys,dt,EulerForward);
            
            nextXef = @(k,x,u)EulerForward.integrate(@(y)sys.f(dt*k,y,u),x,dt);
            
            expX = dtSysEf.f(k,x,u);
            actX = nextXef(k,x,u);
            
            testCase.verifyEqual(expX,actX);
            
        end
    end
    
    
end
