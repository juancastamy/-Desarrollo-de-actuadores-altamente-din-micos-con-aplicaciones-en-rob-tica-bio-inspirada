function f = dynamics(x, u)  
    % Parámetros del sistema
    syms z1 z2 z3 z4 z5 z6 z7
    % Campo vectorial del sistema dinámico
    % q = [x theta] dq = [dx dtheta]
    
    f = [-z1/z2, 0 , -z3*z4/z2; 0, 0, 1; -1/z7, -z4/z7, -z6/z7]*x + [z4/z2; 0; 0]*u; 
end