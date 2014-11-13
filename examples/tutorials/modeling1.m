v = CtSystem( 'StateEquation', @(t,x,u)  [u(1)*cos(x(3));
                                          u(1)*sin(x(3));
                                          u(2)          ], ...
             'nx',3,'nu',2); 
         
         
v = MyUnicycle();