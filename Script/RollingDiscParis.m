% Rolling Disc from Paris
clear all
clc

syms x(t) y(t) psi(t) theta(t) phi(t)
syms X Y PSI THETA PHI DX DY DPSI DTHETA DPHI DDX DDY DDPSI DDTHETA DDPHI real
syms m r g Ia It real
syms lambda1 lambda2

%% 0. Lists
ActualList = [x y psi theta phi diff(x) diff(y) diff(phi) diff(theta) diff(psi)...
                diff(x,t,t) diff(y,t,t) diff(phi,t,t) diff(theta,t,t) diff(psi,t,t)];
HolderList = [X Y PHI THETA PSI DX DY DPHI DTHETA DPSI DDX DDY DDPHI DDTHETA DDPSI];


%% Generelized coordinates

dq = transpose([DX DY DPHI DTHETA DPSI]);

%% 1. Angular velocity kinematics

omega_x = -diff(psi) + diff(phi)*sin(theta);
omega_y = -diff(theta);
omega_z = diff(phi)*cos(theta);

%% 2. Kinetic Energy

T_lin = (1/2) * m * (diff(x)^2 + diff(y)^2 + (r*sin(theta)*diff(theta))^2);
T_rot = (1/2) * (Ia*(omega_x)^2 + It*(omega_y + omega_z)^2);

T = simplify(T_lin + T_rot);

% Mass matrix
T = subs(T,ActualList,HolderList);
[CoeffT,c] = coeffs(T,[DX DY DPHI DTHETA DPSI]);

m11 = CoeffT * transpose([1 0 0 0 0 0 0]);
m22 = CoeffT * transpose([0 1 0 0 0 0 0]);
m33 = CoeffT * transpose([0 0 1 0 0 0 0]);
m34 = CoeffT * transpose([0 0 0 1 0 0 0]);
m35 = CoeffT * transpose([0 0 0 0 1 0 0]);
m44 = CoeffT * transpose([0 0 0 0 0 1 0]);
m55 = CoeffT * transpose([0 0 0 0 0 0 1]);

M = [2*m11 0 0 0 0;
     0 2*m22 0 0 0;
     0 0 2*m33 m34 m35;
     0 0 m34 2*m44 0;
     0 0 m35 0 2*m55];

ShallBeZero = simplify(T - (1/2)*transpose(dq)*M*dq)

%% 4. Potential Energy

V = m*g*r*cos(theta);
V = subs(V,ActualList,HolderList);
dL_q = jacobian(V,[X Y PSI THETA PHI]);

%% 5. Constraints

Q_r = [lambda1;
        lambda2;
        lambda1*r*cos(theta)*sin(phi) - lambda2*r*cos(theta)*cos(phi);
        lambda1*r*sin(theta)*cos(phi) + lambda2*r*sin(theta)*sin(phi);
        -lambda1*r*cos(phi) - lambda2*r*sin(phi)];
    
g1 = simplify(diff(x) - (r*cos(phi)*diff(psi) - r*sin(theta)*cos(phi)*diff(phi) - r*cos(theta)*sin(phi)*diff(theta)));
g2 = simplify(diff(y) - (r*sin(phi)*diff(psi) - r*sin(theta)*sin(phi)*diff(phi) + r*cos(theta)*cos(phi)*diff(theta)));

% Constraint coefficient matrix
g1 = subs(g1,ActualList,HolderList);
A1 = jacobian(g1,[DX DY DPSI DTHETA DPHI]);

g2 = subs(g2,ActualList,HolderList);
A2 = jacobian(g2,[DX DY DPSI DTHETA DPHI]);

%% 6. Equations of Motion
Q_r = subs(Q_r,ActualList,HolderList);
qDDot = simplify(inv(M) * (Q_r + transpose(dL_q)));

xDD_s = [1 0 0 0 0] * qDDot;
yDD_s = [0 1 0 0 0] * qDDot;
phiDD_s = [0 0 1 0 0] * qDDot;
thetaDD_s = [0 0 0 1 0] * qDDot;
psiDD_s = [0 0 0 0 1] * qDDot;

%% Deriving b matrix

g1 = subs(g1,HolderList,ActualList);
g1DD = simplify(diff(g1,t));
g1DD = subs(g1DD,ActualList,HolderList);
[CoeffB1,c] = coeffs(g1DD,[DDX DDY DDPHI DDTHETA DDPSI]);
b1 = CoeffB1 * transpose([0 0 0 0 1]);
b1 = -b1;

g2 = subs(g2,HolderList,ActualList);
g2DD = simplify(diff(g2,t));
g2DD = subs(g2DD,ActualList,HolderList);
[CoeffB2,c] = coeffs(g2DD,[DDX DDY DDPHI DDTHETA DDPSI]);
b2 = CoeffB2 * transpose([0 0 0 0 1]);
b2 = -b2;

%% Lambda

A_tilde = [A1;A2];

%
qDD_tilde = transpose([DDX DDY DDPHI DDTHETA DDPSI]);
qDD_tilde = subs(qDD_tilde,[DDX DDY DDPHI DDTHETA DDPSI],[xDD_s,yDD_s,phiDD_s,thetaDD_s,psiDD_s]);

b_tilde = [b1;b2];

Lambdas = (A_tilde * qDD_tilde == b_tilde);

Eq1 = [1 0] * Lambdas;
Eq2 = [0 1] * Lambdas;

[lambda1_sol,lambda2_sol] = solve([Eq1,Eq2],[lambda1,lambda2]);

%% Constraint forces

Qr_NC = subs(Q_r,[lambda1,lambda2],[lambda1_sol,lambda2_sol]);

Qr_x = simplify([1 0 0 0 0] * Qr_NC)
Qr_y = simplify([0 1 0 0 0] * Qr_NC)
Qr_phi = simplify([0 0 1 0 0] * Qr_NC)
Qr_theta = simplify([0 0 0 1 0] * Qr_NC)
Qr_psi = simplify([0 0 0 0 1] * Qr_NC)





