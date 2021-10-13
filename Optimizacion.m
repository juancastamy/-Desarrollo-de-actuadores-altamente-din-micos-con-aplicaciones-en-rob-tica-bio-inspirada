clear all;
load('DATAREAL.mat')
Z0 = [0.01;0.01;0.01;0.01;0.01;0.01;0.01];
lb = zeros(1,7);
ub = [10,0.01,inf,inf,inf,inf,inf];
%% Parámetros de la simulación
t0 = 0;
tf = 19.16; % tiempo de simulación
dt = 0.01;
N = (tf - t0) / dt;
t = t0:0.01:tf-0.01;
t1 = t';
A = [];
b = [];
Aeq = [];
beq = [];
Z=fmincon(@costfunc,Z0,A,b,Aeq,beq,lb,ub); 

%{
dot_X = @(x,u) ([-Z(1)/Z(2), 0 , -Z(3)*Z(4)/Z(2); 0, 0, 1; -1/Z(7), -Z(4)/Z(7), -Z(6)/Z(7)]*x + [Z(4)/Z(2); 0; 0]*u)

% Arrays para almacenar las trayectorias de las variables de estado,
% entradas y salidas del sistema
%X = zeros(3,N+1);
U = zeros(1,N+1);
% Inicialización de arrays
%X(:,1) = x0;
U(:,1) = u0;
s=[0;0;0];
%% Solución recursiva del sistema dinámico
for n = 0:N-1
    % Método RK4 para la aproximación numérica de la solución
    k1 = dot_X(x(:,n+1),u(n+1));
    k2 = dot_X(x(n+1)+(dt/2)*k1, u(n+1));
    k3 = dot_X(x(n+1)+(dt/2)*k2, u(n+1));
    k4 = dot_X(x(n+1)+dt*k3, u(n+1));
    s = x(n+1) + (dt/6)*(k1+2*k2+2*k3+k4);
    % Se guardan las trayectorias del estado y las entradas
    X(:,n+1) = s;
    U(:,n+1) = u(n+1);
    l=n
end
%}
figure(1);clf;
hold on;
plot(t1,data_Real');

plot(t1,data_Real2');
plot(t1,data_Real3')
