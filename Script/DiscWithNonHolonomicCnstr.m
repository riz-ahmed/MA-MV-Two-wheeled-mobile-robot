% Part 1:
% Deriving Equations of Motions for a disc subjected to Non-holonomic
% Constraints
tic
clear all
close all
clc

% Initialize symbolic variables
syms x(t) z(t) psi(t) theta(t) phi(t)                   % time dependent symblic variables
syms X Z PSI PHI THETA real                             % time independent symbolic variables
syms DX DZ DPSI DTHETA DPHI real                        % first derivaties
syms DDX DDZ DDPSI DDTHETA DDPHI real                   % second derivatices
syms m r g J1 J3 real                                   % symbolic constants
syms lambda_1 lambda_2 real                             % Lagrange Multipliers

%-------------------------------------------------------------------------%
% 0. Lists
%-------------------------------------------------------------------------%

ActualList = [x  z psi phi theta diff(x) diff(z) diff(psi) diff(phi)...
                diff(theta) diff(x,t,t) diff(z,t,t) diff(psi,t,t)...
                diff(phi,t,t)  diff(theta,t,t)];
            
HolderList = [X Z PSI PHI THETA DX DZ DPSI DPHI DTHETA...
                DDX DDZ DDPSI DDPHI DDTHETA];
            

%-------------------------------------------------------------------------%
% 1. Generelized coordinates
%-------------------------------------------------------------------------%

q = transpose([x z psi phi theta]);           % x and y locate the center of the disc in frame I
dq = diff(q);                                 % generelized velocities

%-------------------------------------------------------------------------%
% 2. Kinematics
%-------------------------------------------------------------------------%

% Relating the corotational basis [B1,B2,B3] in the inertial frame of reference [I1,I2,I3]
% using 3-1-3 set of Euler angles

R1 = [cos(psi) -sin(psi) 0;                        % precession [I1 --> F1]
        sin(psi) cos(psi) 0;
        0 0 1];
    
R2 = [1 0 0;                                       % nutation [F1 --> F2]
        0 cos(phi) -sin(phi);
        0 sin(phi) cos(phi)];
    
R3 = [cos(theta) -sin(theta) 0;                    % spin [F2 --> B]
        sin(theta) cos(theta) 0;
        0 0 1];
    
% Express angular velocity in Inertial frame [I1,I2,I3]

omega_I = simplify((R1*R2*R3)*[0;0;diff(theta)] + (R1*R2)*[diff(phi);0;0]...       % Dtheta is negative, initially the disc is assumed to be rolling about -Z
            + (R1) * [0;0;diff(psi)]);
%     % Express angular velocity in corotational basis [B1,B2,B3]
        omega_B = simplify(transpose(R1*R2*R3)*omega_I);
%         
% (Angular velocity about three pricipal axes, expressed in frame I)
omega_x = [1 0 0] * omega_B;
omega_y = [0 1 0] * omega_B;
omega_z = [0 0 1] * omega_B;

%-------------------------------------------------------------------------%
% 2. Kinetic Energy
%-------------------------------------------------------------------------%

T_Rot = (1/2) * (J3 * omega_z^2) + (1/2) * (J1*(omega_x^2 + omega_y^2));    % expressed in I
T_lin = (1/2) * m*(diff(x)^2 + diff(z)^2);                                  % xDot and zDot are directly measured from the disc and expressed in frame I

T = simplify(T_lin + T_Rot);            % Expressed in frame I

% Mass matrix
T = subs(T,ActualList,HolderList);
[CoeffM c] = coeffs(T,[DX DZ DPSI DPHI DTHETA]);
m11 = CoeffM * transpose([1 0 0 0 0 0]);
m22 = CoeffM * transpose([0 1 0 0 0 0]);
m33 = CoeffM * transpose([0 0 1 0 0 0]);
m35 = CoeffM * transpose([0 0 0 1 0 0]);
m44 = CoeffM * transpose([0 0 0 0 1 0]);
m55 = CoeffM * transpose([0 0 0 0 0 1]);
m53 = CoeffM * transpose([0 0 0 1 0 0]);
M = [2*m11 0 0 0 0;
     0 2*m22 0 0 0;
     0 0 2*m33 0 m35;
     0 0 0 2*m44 0;
     0 0 m53 0 2*m55];

dq = subs(dq,ActualList,HolderList);
ShallBeZero = simplify(T - (1/2)*transpose(dq)*M*dq)

%-------------------------------------------------------------------------%
% 3. Potential Energy
%-------------------------------------------------------------------------%

% V = simplify(R1*R2*[0;m*g*r;0])
V = m*g*(r*cos(phi));                              % Expressed in frame B
V = subs(V,ActualList,HolderList);
dL_q = jacobian(V,[X Z PSI PHI THETA]);

%-------------------------------------------------------------------------%
% 4. NonHolonomic Constraints
%-------------------------------------------------------------------------%

% No-slip constraints are applied to P which is located in frame F2
% F2 is fixed to the body but non-rotating

CP_B = transpose(R3) * [0; r; 0];                         % Expressing CP in frame I
vCom = simplify(cross(omega_B,CP_B));               % Vel of CP expressed in frame I

% Velocity of CP along x and z in frame I
Dx1 = simplify([1 0 0]*vCom);
Dx2 = simplify([0 1 0]*vCom);

% Velocity of CP along y (vertical) in frame I
Dx3 = simplify([0 0 1]*vCom);

% Aplying constraints in frame I such that xDot - Dx1 = 0 (No-slip velocity)
% the equations are qritten in the form A(q)qDot = 0
g1 = simplify(diff(x) - (Dx1));                     % xDot - Dx1 = 0
g2 = simplify(diff(z) - (Dx2));                     % zDot - Dx2 = 0

% Deriving A(q): constraint coefficient matrices
g1 = subs(g1,ActualList,HolderList);
A_1 = jacobian(g1,[DX DZ DPSI DPHI DTHETA]);

g2 = subs(g2,ActualList,HolderList);
A_2 = jacobian(g2,[DX DZ DPSI DPHI DTHETA]);

% Constraint forces in total can be written as:
% Q_r = A_1' lambda_1 + A_2' lambda_2
Q_r = transpose(A_1) * lambda_1 + transpose(A_2) * lambda_2;

%-------------------------------------------------------------------------%
% 5. Equations of motion
%-------------------------------------------------------------------------%
% writing in form MqDDot = Q + Q_r
% M_Dot becomes zero
qDDot = simplify(inv(M) * (Q_r + transpose(dL_q)));

qDDot = subs(qDDot,ActualList,HolderList);

xDDot_s = [1 0 0 0 0] * qDDot;
zDDot_s = [0 1 0 0 0] * qDDot;
psiDDot_s = [0 0 1 0 0] * qDDot;
phiDDot_s = [0 0 0 1 0] * qDDot;
thetaDDot_s = [0 0 0 0 1] * qDDot;

%-------------------------------------------------------------------------%
% 6. Writing constraint equation in the form A(q,t)qDDot = b(q,qDot,t)
%-------------------------------------------------------------------------%
% Deriving b_1(q,qDot,t)
g1 = subs(g1,HolderList,ActualList);
g1DDot = simplify(diff(g1,t));
g1DDot = subs(g1DDot,ActualList,HolderList);
[CoeffsB1,c] = coeffs(g1DDot,[DDX DDZ DDPSI DDPHI DDTHETA]);
b1 = CoeffsB1 * transpose([0 0 0 1]);
b1 = -b1;

% Deriving b_2(q,qDot,t)
g2 = subs(g2,HolderList,ActualList);
g2DDot = simplify(diff(g2,t));
g2DDot = subs(g2DDot,ActualList,HolderList);
[CoeffsB2,c] = coeffs(g2DDot,[DDX DDZ DDPSI DDPHI DDTHETA]);
b2 = CoeffsB2 * transpose([0 0 0 1]);
b2 = -b2;

%-------------------------------------------------------------------------%
% 7. Lambda
%-------------------------------------------------------------------------%

% A_tilde = [A_1
%            A_2]
A_tilde = [A_1;A_2];

% qDDot_tilde: Matrix with substituted values of qDDot from Sec:5
qDDot_tilde = transpose([DDX DDZ DDPSI DDPHI DDTHETA]);
qDDot_tilde = subs(qDDot_tilde,[DDX DDZ DDPSI DDPHI DDTHETA],[xDDot_s, zDDot_s, psiDDot_s, phiDDot_s, thetaDDot_s]);

b_tilde = [b1;b2];

% Form the equation A_tilde * qDDot_tilde = b_tilde
Lambdas = (A_tilde * qDDot_tilde == b_tilde);

% Solve the above simultaneous equations for lambda_1 and lambda_2
Eq1 = [1 0]*Lambdas;
Eq2 = [0 1]*Lambdas;
[lambda_1_sol,lambda_2_sol] = solve([Eq1,Eq2],[lambda_1,lambda_2]);

%% Constraint Forces

Qr_NC = subs(Q_r,[lambda_1,lambda_2],[lambda_1_sol,lambda_2_sol]);

Qr_x = simplify([1 0 0 0 0] * Qr_NC)
Qr_z = simplify([0 1 0 0 0] * Qr_NC)
Qr_psi = simplify([0 0 1 0 0] * Qr_NC)
Qr_phi = simplify([0 0 0 1 0] * Qr_NC)
Qr_theta = simplify([0 0 0 0 1] * Qr_NC)



toc
