syms z1 z2 z3 z4 z5 z6 z7 z8 lambda

A = [ -z1/z2,      0, -(z3*z4)/z2;
           0,      0,           1;
       -1/z7, -z5/z7,      -z6/z7];
     
B = [ z4/z2; 0; 0];

C = [ 1/z4, 0, 0;
         0, 1, 0;
         0, 0, 1];

polcarA(lambda) = det(lambda*eye(3)-A)
polcarA(0)
