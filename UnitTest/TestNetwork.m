
classdef TestNetwork < matlab.unittest.TestCase
    
    methods (Test)
        
        function test_netMeasurement(testCase)
            
            node1 = IDtSystem('nx',1,'nu',1,'StateEquation',@(k,x,u,varargin)u);
            node1.controller = IController(@(k,x,netMeasurement) netMeasurement{1}{1}+1);
            node1.initialCondition = 0;
            node2 = IDtSystem('nx',1,'nu',1,'StateEquation',@(k,x,u,varargin)u);
            node2.controller = IController(@(k,x,netMeasurement) netMeasurement{1}{1}+1);
            node2.initialCondition = 0;
            n = 5;
            
            va = VirtualArena({node1,node2},...
                'StoppingCriteria'  ,@(t,as)t>=n,...
                'DefaultPlots',0,...
                'SensorsNetwork'    , {StateSensor(),[0,1;1,0]});
            
            ret = va.run();
            
            testCase.verifyEqual(ret{1}.stateTrajectory(end),n);
            testCase.verifyEqual(ret{2}.stateTrajectory(end),n);
        end
        
    end
end
