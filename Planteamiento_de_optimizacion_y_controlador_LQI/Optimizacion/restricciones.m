function [c, ceq] = restricciones(z)
tol = 0.01;
c = [];
ceq = z(1)*z(5)-tol*z(2)*z(7);