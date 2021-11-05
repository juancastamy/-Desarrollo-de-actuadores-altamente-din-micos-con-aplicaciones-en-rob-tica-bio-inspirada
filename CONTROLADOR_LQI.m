clear all;
load('valores_para_control_LQR.mat','Z','media','mediana');
load('graficas_para_LQI.mat');

A = [-Z(1)/Z(2), 0 , -Z(3)*Z(4)/Z(2);
      0, 0, 1;
     -1/Z(7), -Z(5)/Z(7), -Z(6)/Z(7)];
B = [Z(4)/Z(2); 0; 0];

C = [1/Z(4), 0, 0;
          0, 1, 0;
          0, 0, 1];
D=0;
%{
n = length (A);
rankVal = rank(obsv(A,C));
if (rankVal == n)
    fprintf ('El el sistema es completamente controlable. \n');
else
    fprintf ('El el sistema no es completamente controlable. \n');
end
%}

% x = [cor*Z(4); pos; vel];
% u = [zeros(1,804), ones(1,197)*4095];
% Q=eye(6);
% R=1;
% 
% sys=ss(A,B,C,D);
% K=lqi(sys,Q,R,0);

%{
Klqi=K(1:3);
Ki=K(1,4);
A_cl=A - (B * Klqi);
sys_cl=ss(A_cl,B,C,D);
Nbar = rscale(sys_cl, Klqi);

RL=1;
QL=eye(3);
L=lqr(A,B,QL,RL)';
Abs= A - L*C;
Bbs=[B L];
Cbs=eye(3);
Dbs=[0 0;
     0 0;
     0 0];
Nbar=rscale(sys,Klqi); 
%}

% Matriz de control
n = 3; % numero de variables de estado
m = 1; % numero de entradas

Cr = [1, 0, 0];
pr = 1; % numero de salidas de control

Abar = [A, zeros(n,pr); -Cr, zeros(pr,pr)]
Bbar = [B; zeros(pr, m)]

rank(ctrb(Abar, Bbar))

Qbar = eye(n+pr);
Rbar = eye(m);

Klqi = lqr(Abar, Bbar, Qbar, Rbar)

% K = lqr(A, B, eye(n), eye(m))

