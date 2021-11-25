function sum = costfunc(Z)
    datos = 1;
    if datos == 1
        load('datos_optimizacion2','PULSO','posicion','CORRIENTE','velocidad')
        %% Parámetros de la simulación
        t0 = 0;
        tf = 10; % tiempo de simulación
        dt = 0.001;
        N = 9990;%(tf-t0)/dt;%(tf - t0) / dt;
        t = t0:0.001:tf;
        u=PULSO;
        cor = CORRIENTE';
        pos = posicion';
        vel = velocidad';
    else
        load('DATAREAL.mat')
        %% Parámetros de la simulación
        t0 = 0;
        tf = 20; % tiempo de simulación
        dt = 0.01;
        N = 1915;
        t = t0:0.01:tf-0.01;
        u = data_Real';
        p = zeros(1,1116);

        cor = [p,ones(1,800)*0.18];
        pos = data_Real3';
        vel = data_Real2';
    end
    
    
    %% Definicion de las matrices del sistema
    A = [-Z(1)/Z(2),         0 , -Z(3)*Z(4)/Z(2);
                  0,          0,               1;
            -1/Z(7), -Z(5)/Z(7),      -Z(6)/Z(7)];
        
    B = [Z(4)/Z(2); 0; 0];
    
    C = [1/Z(4), 0, 0;
              0, 1, 0;
              0, 0, 1];    

    real = [cor;pos;vel];
    
    [X,Y] = simulate(A, B, C, u, N, dt);
    sum = 0;
    for i = 0:N
        sum = sum + norm(Y(:,(i+1)) - real(:,i+1));
    end
end

