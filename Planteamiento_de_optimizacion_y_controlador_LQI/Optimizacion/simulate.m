function [X,Y] = simulate(A, B, C, u, N, dt)
    %% definicion de las funciones del sistema
    dot_X = @(x,u) (A*x + B*u);
    %se inicializan las matrices de salida
    X = zeros(3,N);
    Y = zeros(3,N);
    
    x0 = [0;0;0];
    x = x0;
    
    X(:,1) = x0;
    Y(:,1) = C*x0;    
    
    %% se emprea el metodo de RK4
    for n = 0:N
        % Método RK4 para la aproximación numérica de la solución
        k1 = dot_X(x,u(n+1));
        k2 = dot_X(x+(dt/2)*k1, u(n+1));
        k3 = dot_X(x+(dt/2)*k2, u(n+1));
        k4 = dot_X(x+dt*k3, u(n+1));
        x = x + (dt/6)*(k1+2*k2+2*k3+k4);
        y = C*x;
        % Se guardan las trayectorias del estado y las entradas
        X(:,n+1) = x;
        Y(:,n+1) = y;
    end
    
end

