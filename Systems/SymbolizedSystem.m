
classdef SymbolizedSystem < handle
    
    properties
        nameFfile
        nameHfile
    end
    
    methods (Abstract) 
        h(obj,t,x,u)
        f(obj,t,x,u)
    end
    methods
        
        function obj = SymbolizedSystem (sys)
            
            
            t = sym('t',[1,1]);
            x = sym('x',[sys.nx,1]);
            u = sym('u',[sys.nu,1]);
            
            
            if isempty(which('assume'))
                t = sym(t,'real');
                x = sym(x,'real');
                u = sym(u,'real');
            else
                assume(t,'real');
                assume(x,'real');
                assume(u,'real');
            end
            
            addpath './gen';
            if not(exist('./gen', 'dir') == 7)
                mkdir ./gen;
            end
            
            %% Generate function for h
            % Find first name available
            nameIndex = 1;
            while exist(sprintf('./gen/%s%i_f.m',class(sys),nameIndex)) == 2
                nameIndex = nameIndex+1;
            end
            
            newFileName = sprintf('./gen/%s%i_f.m',class(sys),nameIndex);
            
            matlabFunction(simplify( sys.f(t,x,u) ) ,'vars',{t,x,u},'File',newFileName);
            
            obj.nameFfile = sprintf('%s%i_f',class(sys),nameIndex);
            
            %% Generate function for f
            % Find first name available
            nameIndex = 1;
            while exist(sprintf('./gen/%s%i_h.m',class(sys),nameIndex)) == 2
                nameIndex = nameIndex+1;
            end
            
            newFileName = sprintf('./gen/%s%i_h.m',class(sys),nameIndex);
            
            matlabFunction(simplify( sys.h(t,x,u) ) ,'vars',{t,x,u},'File',newFileName);
            
            obj.nameHfile = sprintf('%s%i_h',class(sys),nameIndex);
            
        end
        
        
    end
end