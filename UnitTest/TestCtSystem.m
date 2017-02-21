
classdef TestCtSystem < matlab.unittest.TestCase
    
    methods (Test)
        
        function test_CtSystem_getTrajectories(testCase)
            
            sys   = MyCtSystem();
            
            TestCtSystem.addtest_getTrajectories(sys,testCase);
            
        end
        
        function test_CtSystem_getStateTrajectory(testCase)
            
            sys   = MyCtSystem();
            
            TestCtSystem.addtest_getStateTrajectory(sys,testCase);
            
        end
        
        
         function test_CtSystem_getStateTrajectoryLaw(testCase)
            
            sys   = MyCtSystem();
            
            TestCtSystem.addtest_getStateTrajectoryLaw(sys,testCase);
            
         end
         
        
    end
    
    methods (Static)
        function addtest_getTrajectories(sys,testCase)
            
            dt    = 1;
            xNext = @(k,x,u)RK4.integrate(@(x)sys.f(k,x,u),x,dt);
            
            u  = [1,2];
            x0 = [1;2];
            t0 = 1;
            
            law = @(k,x) -x(1)*k+x(2);
            N  = 2;
            nu = 1;
            
            [actTout,actXout,actUout] = sys.getTrajectories(t0,x0,law,dt,N,nu);
            
            x1 = xNext(t0   , x0, law(t0,x0));
            u1 = law(t0,x0);
            x2 = xNext(t0+dt, x1, law(t0+dt,x1));
            u2 = law(t0+dt,x1);
            
            expXout = [x0,x1,x2];
            expUout = [u1,u2];
            expTout = [t0,t0+dt,t0+2*dt];
            
            testCase.verifyEqual(actXout,expXout);
            testCase.verifyEqual(actUout,expUout);
            testCase.verifyEqual(actTout,expTout);
        end
        
        
        function addtest_getStateTrajectoryLaw(sys,testCase)
            
            dt    = 1;
            xNext = @(k,x,u)RK4.integrate(@(x)sys.f(k,x,u),x,dt);
            
            u  = [1,2];
            x0 = [1;2];
            t0 = 1;
            
            law = @(k,x) -x(1)*k+x(2);
            N = 2;
            actXout = sys.getStateTrajectoryLaw(t0,x0,law,dt,N);
            x1 = xNext(t0   , x0, law(t0,x0));
            x2 = xNext(t0+dt, x1, law(t0+dt,x1));
            
            expXout = [x0,x1,x2];
            
            testCase.verifyEqual(actXout,expXout);
        end
        
        function addtest_getStateTrajectory(sys,testCase)
            
            dt    = 0.5;
            xNext = @(k,x,u)RK4.integrate(@(x)sys.f(k,x,u),x,dt);
            
            u  = [1,2];
            x0 = [1;2];
            t0 = 1;
            
            actXout = sys.getStateTrajectory(t0,x0,u,dt);
            expXout = [x0,...
                       xNext(t0   ,x0               ,u(1)),...
                       xNext(t0+dt,xNext(t0,x0,u(1)),u(2))];
            
            testCase.verifyEqual(actXout,expXout);
            
        end
        
    end
end 
