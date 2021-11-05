clear all;
op=1;
%load('Valores_impulso_unitario_para_realizar_graficas_correctas.mat');
if op ==1
    Z0 = [0.01;0.01;0.01;0.01;0.01;0.01;0.01]*0;
    lb = zeros(1,7); lb(4)=0.065;
    ub = [10,0.01,inf,0.065,inf,1,1];
    %% Parámetros de la simulación
    t0 = 0;
    tf = 20; % tiempo de simulación
    dt = 0.01;
    %N = (tf - t0) / dt;
    t = t0:0.01:tf-0.01;
    t1 = t';
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    Z=fmincon(@costfunc,Z0,A,b,Aeq,beq,lb,ub,@restricciones)
else
    load('Z1.mat');
    Z1 = Z';
    
    load('Z2.mat');
    Z2 = Z';
    
    load('Z3.mat');
    Z3 = Z';
    
    load('Z4.mat');
    Z4 = Z';
    
    load('Z5.mat');
    Z5 = Z';
    
    load('Z6.mat');
    Z6 = Z';
    
    load('Z7.mat');
    Z7 = Z';
    
    load('Z8.mat');
    Z8 = Z';
    
    load('Z9.mat');
    Z9 = Z';
    
    load('Z10.mat');
    Z10 = Z';
    
    load('Z11.mat');
    Z11 = Z';
    
    load('Z12.mat');
    Z12 = Z';
    
    Z = [Z1;Z2;Z3;Z4;Z5;Z6;Z7;Z8;Z9;Z10;Z11;Z12];
    media = mean(Z);
    mediana = median(Z);
    save('valores_para_control_LQR.mat','Z','media','mediana');
end

