
classdef TestFractionalOrder < matlab.unittest.TestCase
    
    methods (Test)
        
        function test_Ass1(testCase)
            
            sys = DtFractionalOrderSystem([1],[0],{eye(2)},{[1;3]});
            testCase.verifyEqual(true,sys.satisfiesAss1());
            sys = DtFractionalOrderSystem([1],[0],{[1,0;0,0]},{[1;3]});
            testCase.verifyEqual(false,sys.satisfiesAss1());
            
        end
    end
    
    
end
