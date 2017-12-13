
classdef TestUtils < matlab.unittest.TestCase
    
    methods (Test)
        
        function test_MinMaxKeeper(testCase)
            
            a = MinMaxKeeper();
            a.add(1);
            a.add(2);
            a.add(3);
            
            testCase.verifyEqual(1,a.getMin);
            testCase.verifyEqual(3,a.getMax);
            
            a = MinMaxKeeper();
            a.add(1);
            a.add(2);
            a.addMin(3);
            
            testCase.verifyEqual(1,a.getMin);
            testCase.verifyEqual(2,a.getMax);
            
            a = MinMaxKeeper();
            
            a.add(2);
            a.addMax(1);
            
            testCase.verifyEqual(2,a.getMin);
            testCase.verifyEqual(2,a.getMax);
            
            
        end
        
        function test_expandInputs(testCase)
            
            inputSizes = {2,{1},{3,1}};
            
            xSym = Symbolizer.getSymOfCell (inputSizes,'x',0);
            vars = Symbolizer.expandInputs(xSym);
            
            varsDes = {sym('x1',[inputSizes{1},1]),...
                sym('x2_1',[inputSizes{2}{1},1]),...
                sym('x3_1',[inputSizes{3}{1},1]),...
                sym('x3_2',[inputSizes{3}{2},1])};
            
            testCase.verifyEqual(varsDes,vars);
            xSymExp = Symbolizer.getSymOfCell (inputSizes,'x',1);
            testCase.verifyEqual(varsDes,xSymExp);
            
        end
        
        function test_symbolizeEvaluateFunction(testCase)
            inputSizes = {1,{1},{1,1}};
            
            xSym = Symbolizer.getSymOfCell (inputSizes,'x',0);
            
            symFun = Symbolizer.symbolize(@(a,b,c)TestUtils.testFunction(a,b,c), inputSizes);
            
            testCase.verifyEqual(TestUtils.testFunction(xSym{:}),symFun(xSym{:}));
        end
        
        function test_Symbolizer_getSymOfCell(testCase)
            
            inputSizes = {1,2,3,{1,{2,3}}};
            
            vars = Symbolizer.getSymOfCell (inputSizes,'x',1);
            
            varsDes = {sym('x1',[inputSizes{1},1]),...
                sym('x2',[inputSizes{2},1]),...
                sym('x3',[inputSizes{3},1]),...
                sym('x4_1',[inputSizes{4}{1},1]),...
                sym('x4_2_1',[inputSizes{4}{2}{1},1]),...
                sym('x4_2_2',[inputSizes{4}{2}{2},1])};
            
            testCase.verifyEqual(varsDes,vars);
            
            vars = Symbolizer.getSymOfCell (inputSizes,'x',0);
            
            varsDes = {sym('x1',[inputSizes{1},1]),...
                sym('x2',[inputSizes{2},1]),...
                sym('x3',[inputSizes{3},1]),...
                {
                sym('x4_1',[inputSizes{4}{1},1]),...
                {
                sym('x4_2_1',[inputSizes{4}{2}{1},1]),...
                sym('x4_2_2',[inputSizes{4}{2}{2},1])},...
                }};
            
            testCase.verifyEqual(varsDes,vars);
            
            
        end
        function test_Symbolizer_symbolize(testCase)
            
            f = @(a,b,c) a + b + c{1} + c{2};
            
            inputSizes = {1,1,{1,1}};
            
            symF = Symbolizer.symbolize(f, inputSizes );
            
            args = {21,3,{7,13}};
            
            
            vDes = f(args{:});
            v = symF(args{:});
            testCase.verifyEqual(vDes,v);
            
        end
    end
    
    methods (Static)
        function ret = testFunction(a,b,c)
            ret = a + b{1} + c{1} + c{2};
        end
    end
    
    
end
