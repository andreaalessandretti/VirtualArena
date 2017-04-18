

classdef LinearizedSystem < handle
    
    properties
        % x(k+1)/dot(x) = A x(k) + B u(k) + p(k)
        % y(k)          = C x(k) + D u(k) + q(k)
        
        symbolicLinarization = 0;
        samplesRes = 0.0001;
        nameFunA
        nameFunB
        nameFunp
        nameFunC
        nameFunD
        nameFunq
        
    end
    
    methods (Abstract)
        h(obj,t,x,u)
        f(obj,t,x,u)
    end
    methods
        
        function ret = A(obj,tbar,xbar,ubar)
            if obj.symbolicLinarization
                ret = feval(obj.nameFunA,tbar,xbar,ubar);
            else
                ret = LinearizedSystem.jacobianSamples(@(x)obj.f(tbar,x,ubar),xbar,obj.samplesRes);
            end
        end
        function ret = B(obj,tbar,xbar,ubar)
            if obj.symbolicLinarization
                ret = feval(obj.nameFunB,tbar,xbar,ubar);
            else
                ret = LinearizedSystem.jacobianSamples(@(u)obj.f(tbar,xbar,u),ubar,obj.samplesRes);
            end
            
        end
        function ret = p(obj,t,tbar,xbar,ubar)
            if obj.symbolicLinarization
                ret = feval(obj.nameFunp,t,tbar,xbar,ubar);
            else
                DtF   = @(tbar,xbar,ubar) LinearizedSystem.jacobianSamples(@(t)obj.f(t,xbar,ubar),tbar,obj.samplesRes);
                
                ret = ( DtF(tbar,xbar,ubar)*t + obj.f(tbar,xbar,ubar) - [DtF(tbar,xbar,ubar),obj.A(tbar,xbar,ubar),obj.B(tbar,xbar,ubar)]*[tbar;xbar;ubar] );
            end
            
        end
        function ret = C(obj,tbar,xbar,ubar)
            if obj.symbolicLinarization
                ret = feval(obj.nameFunC,tbar,xbar,ubar);
            else
                ret = LinearizedSystem.jacobianSamples(@(x)obj.h(tbar,x,ubar),xbar,obj.samplesRes);
            end
        end
        function ret = D(obj,tbar,xbar,ubar)
            if obj.symbolicLinarization
                ret = feval(obj.nameFunD,tbar,xbar,ubar);
            else
                ret = LinearizedSystem.jacobianSamples(@(u)obj.h(tbar,xbar,u),ubar,obj.samplesRes);
            end
            
        end
        function ret = q(obj,t,tbar,xbar,ubar)
            if obj.symbolicLinarization
                ret =  feval(obj.nameFunq,t,tbar,xbar,ubar);
            else
                DtH   = @(tbar,xbar,ubar) LinearizedSystem.jacobianSamples(@(t)obj.h(t,xbar,ubar),tbar,obj.samplesRes);
                
                ret = ( DtH(tbar,xbar,ubar)*t + obj.h(tbar,xbar,ubar) - [DtH(tbar,xbar,ubar),obj.C(tbar,xbar,ubar),obj.D(tbar,xbar,ubar)]*[tbar;xbar;ubar] );
            end
        end
        
        function useLinearizationBySamples(obj)
            obj.symbolicLinarization = 0;
        end
        function useSymbolicLinearization(obj)
            
            obj.symbolicLinarization = 1;
            
            if isempty(obj.nameFunA)
                addpath './gen';
                if not(exist('./gen', 'dir') == 7)
                    mkdir ./gen;
                end
                
                %% Compute linearizations
                t = sym('t',[1,1]);
                
                tbar = sym('tbar',[1,1]);
                
                xbar = sym('xbar',[obj.nx,1]);
                
                ubar = sym('ubar',[obj.nu,1]);
                
                
                if isempty(which('assume'))
                    ubar = sym(ubar,'real');
                    xbar = sym(xbar,'real');
                    tbar = sym(tbar,'real');
                    t    = sym(t,'real');
                else
                    assume(ubar,'real');
                    assume(xbar,'real');
                    assume(tbar,'real');
                    assume(t,'real');
                end
                
                fprintf(getMessage('DynamicalSystem:LinearizingStateEquation'));
                
                DtF   = matlabFunction(  jacobian(obj.f(tbar,xbar,ubar),tbar), 'vars', {tbar,xbar,ubar});
                
                [newFileName,newFunctionName] = LinearizedSystem.selectFileNames(class(obj),'A');
                
                matlabFunction(  jacobian(obj.f(tbar,xbar,ubar),xbar), 'vars', {tbar,xbar,ubar},'File',newFileName);
                
                obj.nameFunA = newFunctionName;
                
                
                [newFileName,newFunctionName] = LinearizedSystem.selectFileNames(class(obj),'B');
                
                matlabFunction(  jacobian(obj.f(tbar,xbar,ubar),ubar), 'vars', {tbar,xbar,ubar},'File',newFileName);
                
                obj.nameFunB = newFunctionName;
                
                
                
                [newFileName,newFunctionName] = LinearizedSystem.selectFileNames(class(obj),'p');
                
                matlabFunction( DtF(tbar,xbar,ubar)*t + obj.f(tbar,xbar,ubar) - [DtF(tbar,xbar,ubar),obj.A(tbar,xbar,ubar),obj.B(tbar,xbar,ubar)]*[tbar;xbar;ubar],'vars',{t,tbar,xbar,ubar},'File',newFileName);
                
                obj.nameFunp = newFunctionName;
                
                fprintf(getMessage('done'));
                
                
                
                fprintf(getMessage('DynamicalSystem:LinearizingOutputEquation'));
                
                
                
                DtH   = matlabFunction(  jacobian(obj.h(tbar,xbar,ubar),tbar)  ,'vars',{tbar,xbar,ubar});
                
                [newFileName,newFunctionName] = LinearizedSystem.selectFileNames(class(obj),'C');
                
                matlabFunction(  jacobian(obj.h(tbar,xbar,ubar),xbar)  ,'vars',{tbar,xbar,ubar},'File',newFileName);
                
                obj.nameFunC = newFunctionName;
                
                
                [newFileName,newFunctionName] = LinearizedSystem.selectFileNames(class(obj),'D');
                
                matlabFunction(  jacobian(obj.h(tbar,xbar,ubar),ubar)  ,'vars',{tbar,xbar,ubar},'File',newFileName);
                
                obj.nameFunD = newFunctionName;
                
                
                [newFileName,newFunctionName] = LinearizedSystem.selectFileNames(class(obj),'q');
                
                matlabFunction( DtH(tbar,xbar,ubar)*t + obj.h(tbar,xbar,ubar) - [DtH(tbar,xbar,ubar),obj.C(tbar,xbar,ubar),obj.D(tbar,xbar,ubar)]*[tbar;xbar;ubar]   ,'vars',{t,tbar,xbar,ubar},'File',newFileName);
                
                obj.nameFunq = newFunctionName;
                
                fprintf(getMessage('done'));
            end
            
            
        end
        
        
        
    end
    
    methods (Static)
        
        
        
        function [newFileName,newFunctionName] = selectFileNames(systemClass,nameMatrix)
            
            nameIndex = 1;
            while exist(sprintf('./gen/lin%s%i_%s.m',systemClass,nameIndex,nameMatrix)) == 2
                nameIndex = nameIndex+1;
            end
            
            newFileName = sprintf('./gen/lin%s%i_%s.m',systemClass,nameIndex,nameMatrix);
            
            newFunctionName = sprintf('lin%s%i_%s',systemClass,nameIndex,nameMatrix);
            
        end
        
        % e.g.
        %f = @(x,u)x^2*u+x-u;
        %xbar =1;ubar =1;
        %A = jacobianSamples(@(x)f(x,ubar),xbar)
        %B = jacobianSamples(@(u)f(xbar,u),ubar)
        
        function D = jacobianSamples(f,xbar,a)
            
            if size(f(0.1*ones(size(xbar))),1)==1
                f=@(x)f(x)';
            end
            
            
            nx = length(xbar);
            
            dx = a*eye(nx);
            invdxdxt = (1/a^2)*eye(nx); % inv(dx*dx')
            celldx = mat2cell(dx,nx,ones(1,nx));
            dfc = arrayfun(@(dx)( f(xbar+dx{:})-f(xbar) ),celldx,'UniformOutput',0);
            df = cell2mat(dfc);
            D = (df*dx')*invdxdxt';
            
        end
        
        % function D = jacobianSamples(f,xbar,varargin)
        %  if nargin >2
        %      a = varargin{1};
        %  else
        %      a = 0.1;
        %  end
        %     nx = length(xbar);
        %     dx = a*eye(nx);
        %     invdxdxt = (1/a^2)*eye(nx); % inv(dx*dx')
        %     celldx = mat2cell(dx,nx,ones(1,nx));
        %     dfc = arrayfun(@(dx)f(xbar+dx{:})'-f(xbar)',celldx,'UniformOutput',0);
        %     df = cell2mat(dfc);
        %
        %
        %     D = zeros(1,nx);
        %
        %    for j = 1:1000
        %      for i=1:nx
        %          y = df(:,i);
        %          s = dx(:,i);
        %
        %          D = D + dx(:,i)'*df(:,i)/norm(dx(:,i))^2;
        %
        %      end
        %    end
        %     D = (df*dx')*invdxdxt';
        %
        % end
        
    end
    
end