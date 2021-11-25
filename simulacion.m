%% Parámetros del sistema
%load('valores_para_control_LQR2.mat');
load('Z_carga','Z');
%% Matrices continuas del sistema LTI
A = [ -Z(1)/Z(2),         0, -Z(3)*Z(4)/Z(2);
               0,         0,               1;
         -1/Z(7), -Z(5)/Z(7),     -Z(6)/Z(7)];
     
B = [ Z(4)/Z(2); 0; 0];

C = [ 1/Z(4), 0, 0;
           0, 1, 0;
           0, 0, 1];
D = 0;

%% Parámetros de la simulación
t0 = 0; % tiempo inicial
tf = 10; % tiempo de simulación
dt = 0.001; % período de muestreo
K = (tf - t0) / dt; % número de iteraciones

%% Inicialización y condiciones iniciales
x0 = [0; 0; 0]; % condiciones iniciales 
u0 = 0;
y0 = 0;
x = x0; % vector de estado
u = u0; % entrada
y = y0; % salida
% Array para almacenar las trayectorias de las variables de estado
X = zeros(numel(x0), K+1); X(:,1) = x;
% Array para almacenar la evolución de las entradas 
U = zeros(numel(u0), K+1); U(:,1) = u; 
% Array para almacenar la evolución de las salidas
Y = zeros(1, K+1); Y(:,1) = y; 
% Array para almacenar la evolución de la referencia
R = zeros(1, K+1); 

%% Discretización del sistema continuo
% Discretizamos el sistema empleando ZOH
sys = ss(A,B,C,0);
sysd = c2d(sys, dt, 'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;

%% Diseño de control
% Prueba LQR
Q = eye(3); Q(2,2) = 10;
R = 0.25;
Klqr = lqr(A, B, Q, R);

% Diseño LQI
Cr = [0, 1, 0];
Abar = [A, zeros(3,1); -Cr, 0];
Bbar = [B; 0];
ver = rank(ctrb(Abar,Bbar));
if ver == length(Abar)
    fprintf ('El sistema es completamente controlable. \n');
else 
    fprintf ('El sistema no es completamente controlable. \n');
end
    
Qbar = eye(4); %Qbar(4,4) = 200; Qbar(3,3) = 0.5; 
Qbar(4,4) = 100;
Rbar = 0.1;
Klqi = lqr(Abar, Bbar, Qbar, Rbar);
% Klqi = [300 200 -1 500];
% Klqi = place(Abar, Bbar, [-0.1,-0.2,-0.3,-0.4]);
xI = 0;
tau_d = 0.73;

%% Pseudocódigo para la Tiva
% k1 = Klqi(1) = 0.2411
% k2 = Klqi(2) = -0.9762
% k3 = Klqi(3) = -0.5061
% k4 = Klqi(4) = -20.0000
% error_tau = tau_d - corriente_sensada*k_tau
% xI = xI + error_tau*dt;
% u = 2*(-k1*torque[Nm] -k2*posición[rad] -k3*velocidad[rad/s] - 10*k4*xI); 
% PWM = (abs(u)/12)*100% o *rango etc..

%% Solución recursiva del sistema dinámico
for k = 1:K
    % Escalón unitario
    %r = tau_d*sin(2*pi*0.1*k*dt); % referencia de torque
    %r = exp(-0.2*dt*k)*tau_d*sin(2*pi*0.1*k*dt); 
    r = pi/2;%tau_d;
    
    xI = xI + (r - x(2))*dt;
%   u = -2*Klqi*[x; 6*xI];
   u = -Klqi*[x; xI];
%     u = -Klqr*x;
%     u = 1;
    %u = -1.83037700777899;
    
    
    % Se propaga el sistema LTI discreto para aproximar la solución del
    % sistema continuo
    x = Ad*x + Bd*u;
    
    % Generación de la señal de salida
    y = Cr*x;
    
    % Se guardan las trayectorias del estado, entrada y salida
    X(:, k+1) = x;
    U(:, k+1) = u;
    Y(:, k+1) = y;
    R(:, k+1) = r;
end

U(:, 1) = U(:, 2);
Y(:, 1) = Y(:, 2);
R(:, 1) = R(:, 2);

%% Animación y generación de figuras (NO modificar)
t = t0:dt:tf; % vector de tiempo para generar las gráficas

% Evolución de las variables de estado en el tiempo
figure;
subplot(2,2,1);
plot(t, X', 'LineWidth', 1);
grid minor;
xlabel('$t$', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('$\mathbf{x}(t)$', 'Interpreter', 'latex', 'FontSize', 16);

% Evolución de las entradas en el tiempo
subplot(2,2,2);
plot(t, U', 'LineWidth', 1);
grid minor;
xlabel('$t$', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('$u(t)$', 'Interpreter', 'latex', 'FontSize', 16);
% ylim([-1,1]);

% Evolución de las salidas en el tiempo
subplot(2,2,3);
plot(t, R', '--', 'LineWidth', 1);
hold on;
plot(t, Y', 'LineWidth', 1);
grid minor;
xlabel('$t$', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('$y(t)$', 'Interpreter', 'latex', 'FontSize', 16);