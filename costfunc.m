function sum = costfunc(Z)
    load('DATAREAL.mat')
    %% Parámetros de la simulación
    t0 = 0;
    tf = 19.15; % tiempo de simulación
    dt = 0.01;
    N = (tf - t0) / dt;
    t = t0:0.01:tf-0.01;
    
    %% Definicion de las matrices del sistema
    A = [-Z(1)/Z(2), 0 , -Z(3)*Z(4)/Z(2);
          0, 0, 1;
         -1/Z(7), -Z(5)/Z(7), -Z(6)/Z(7)];
    B = [Z(4)/Z(2); 0; 0];
    
    C = [1/Z(4), 0, 0;
              0, 1, 0;
              0, 0, 1];    
    
    u = data_Real';
    p = zeros(1,1116);
    
    cor = [p,ones(1,800)*0.8];
    pos = data_Real3';
    vel = data_Real2';
    
    real = [cor;pos;vel];
    
    [X,Y] = simulate(A, B, C, u, N, dt);
    sum = 0;
    for i = 0:N
        sum = sum + norm(Y(:,(i+1)) - real(:,i+1));
    end
end

