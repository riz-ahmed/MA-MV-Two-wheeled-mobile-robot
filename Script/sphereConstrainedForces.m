clear all
clc

syms x(t) y(t) psi(t) phi(t) theta(t)
syms X Y PSI PHI THETA DX DY DPSI DPHI DTHETA DDX DDY DDPSI DDPHI ...
     DDTHETA real
syms m R J real
syms Fx Fy Tx Ty Tz

% Lists
ActualList = [x(t) y(t) psi(t) phi(t) theta(t) diff(x) diff(y) diff(psi) diff(phi) diff(theta) ...
              diff(x,t,t) diff(y,t,t) diff(psi,t,t) diff(phi,t,t) diff(theta,t,t)];
HolderList = [X Y PSI PHI THETA DX DY DPSI DPHI DTHETA DDX DDY DDPSI DDPHI DDTHETA];

% M

M = [m 0 0 0 0;
     0 m 0 0 0;
     0 0 2*J 0 -2*J*sin(phi);
     0 0 0 J 0;
     0 0 -2*J*sin(phi) 0 2*J];
M = subs(M,ActualList,HolderList); 
% Q
Q = [Fx;
     Fy;
     Tx + 2*J*diff(phi)*cos(phi)*diff(theta);
     Ty - J*diff(psi)*cos(phi)*diff(theta);
     2*Tx + 2*J*diff(phi)*cos(phi)*diff(psi)];
Q = subs(Q,ActualList,HolderList); 
% A1 and A2
A1 = [1 0 0 0 -R*cos(psi)];
A2 = [0 1 0 0 -R*sin(psi)];
A1 = subs(A1,ActualList,HolderList); 
A2 = subs(A2,ActualList,HolderList);
% B1 and B2
b1 = -R*diff(theta)*sin(psi);
b2 = R*diff(theta)*cos(psi);
b1 = subs(b1,ActualList,HolderList); 
b2 = subs(b2,ActualList,HolderList);

% lambda
lambda1 = simplify(-inv(A1*inv(M)*transpose(A1)) * (b1 - A1*inv(M)*Q));
lambda2 = simplify(-inv(A2*inv(M)*transpose(A2)) * (b2 - A2*inv(M)*Q));

% Forces
Qr = simplify(-transpose(A1)*lambda1 - transpose(A2) * lambda2);
Qr1 = [1 0 0 0 0] * Qr
Qr2 = [0 1 0 0 0] * Qr
Qr3 = [0 0 1 0 0] * Qr
Qr4 = [0 0 0 1 0] * Qr
Qr5 = [0 0 0 0 1] * Qr








