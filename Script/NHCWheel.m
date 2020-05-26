% Solving for the Equations of motion of a disk rolling on a flat plane
% and determining the NC constraint forces (Equality) that satify the 
% no-slip constraint of the rolling disk near the contact point
tic
clear all
clc
% Intializing symbolic variables as functions of time
syms X(t) Y(t) psi(t) theta(t) phi(t)
% syms dX dY dpsi dtheta dphi real
syms m g R Jt Ja lambda_1 lambda_2
assume((m > 0) & (g > 0) & (R > 0) & (Jt > 0) & (Ja > 0))

% Generalized Coordinates for a Nonholonomic system
% Constraints: {T_z == 0, Slip_X == Slip_Y == 0}
q = [X; Y; psi; theta; phi]
dq = diff(q,t) %= [dX dY dpsi dtheta dphi]'

% Nonholonomic Constraints on velocity for no-slip condition
C_T_1 = transpose([1 0 0 0 -R*cos(phi)])
C_T_2 = transpose([0 1 0 0 -R*sin(phi)])

% Angular velocity kinematics:
% I: {Inertial frame}
% B: {Body fixed frame}
% Relate the space-fixed basis {I1,I2,I3} to the corotational basis
% {B1,B2,B3} using a 3-1-3 set of Euler angles:

R1 = [cos(psi), sin(psi), 0;        
      -sin(psi), cos(psi), 0;
      0, 0, 1]; 
  
R2 = [1, 0, 0;                    	
      0, cos(theta), sin(theta);
      0, -sin(theta), cos(theta)];

R3 = [cos(phi), sin(phi), 0;        
      -sin(phi), cos(phi), 0;
      0, 0, 1];  
% Express the angular velocity vector in terms of {B1,B2,B3}:  
omega = simplify((R3*R2*R1)*[0; 0; diff(psi)] + (R3*R2)*[diff(theta); 0; 0] + R3*[0; 0; diff(phi)]);

% Derive angular velocities for axis {B1,B2,B3}
omega_1 = transpose(omega)*transpose([1 0 0])
omega_2 = transpose(omega)*transpose([0 1 0])
omega_3 = transpose(omega)*transpose([0 0 1])

% Deriving Equations of motion in Lagrangian Formalism

% Linear velocity of the disk's mass center in generelized coordinates
vSquared = (diff(X,t))^2 + (diff(Y,t))^2 + R^2*sin(theta).^2*(diff(theta,t))^2

% Total KE of the system
T = 1/2*m*vSquared + Ja/2*(omega_3)^2 + Jt/2*(omega_1^2) + Jt/2*(omega_2^2)
T_exp = expand(T);
T_exp = simplify(T_exp)

% Mass matrix from writing KE in generelized coordinates
[CoeffX c] = coeffs(T_exp,[diff(X,t) diff(Y,t) diff(psi,t) diff(theta,t) diff(phi,t)])
m11 = CoeffX * transpose([1 0 0 0 0 0]);
m22 = CoeffX * transpose([0 1 0 0 0 0]);
m33 = CoeffX * transpose([0 0 1 0 0 0]);
m35 = CoeffX * transpose([0 0 0 1 0 0]);
m44 = CoeffX * transpose([0 0 0 0 1 0]);
m53 = m35;
m55 = CoeffX * transpose([0 0 0 0 0 1]);

% Mass matrix M = M(q)
M = [2*m11 0 0 0 0; 0 2*m22 0 0 0; 0 0 2*m33 0 m35; 0 0 0 2*m44 0; 0 0 m53 0 2*m55]
% Test the mass matrix
% dq = (diff(q,t))';
% M = buildQuadraticForm(T_exp,dq)

%
% ActualList = [diff(X,t) diff(Y,t) diff(psi,t) diff(theta,t) diff(phi,t)];
% HolderList = [dX dY dpsi dtheta dphi]
% T_exp_Holder = subs(T_exp, ActualList, HolderList)
% dq_Holder = subs(dq, ActualList, HolderList)
ShallBeZero = simplify(T - 0.5*transpose(dq)*M*dq)

% Potential Energy of the system
V = m*g*R*cos(theta)

% Equations of motion (qDDot) with constraints and NC Forces
M_inv = inv(M);
DM = diff(M,t)
qDot = diff(q,t)
dVdt = diff(V,t)
C_tilde = C_T_1*lambda_1 + C_T_2*lambda_2

% Total acceleration of the sytem
qDDot = M_inv*[-DM*(qDot) + dVdt + C_tilde]

% Lambda from Constraints
ct_1 = (diff(X,t) == R*diff(phi,t)*cos(psi))
ct_2 = (diff(Y,t) == R*diff(phi,t)*sin(psi))

% Diffenrentiating the constaints
ct_1_l1 = diff(ct_1,t)
ct_2_l2 = diff(ct_2,t)

% Constraints acceleration coefficient matrix (Q_tilde)
[Q1, q1] = coeffs(rhs(ct_1_l1),[diff(X,t,t) diff(Y,t,t) diff(psi,t,t) diff(theta,t,t) diff(phi,t,t)])
Q11 = Q1 * transpose([1 0])
% Q15 = Q1 * transpose([0 1])
[Q2, q2] = coeffs(rhs(ct_2_l2),[diff(X,t,t) diff(Y,t,t) diff(psi,t,t) diff(theta,t,t) diff(phi,t,t)])
Q22 = Q2 * transpose([1 0])
% Q25 = Q2 * transpose([0 1])

Q_tilde = [1 0 0 0 Q11; 0 1 0 0 Q22]

% Constraints value matrix (B_tilde)
B1 = Q1 * transpose([0 1])
B2 = Q2 * transpose([0 1])
B_tilde = [B1; B2]

% Substituting qDDot into the derivative of constraints [Q_tilde*qDDot = W_tilde]
C_T = [C_T_1  C_T_2]
Q_tilde_new = simplify(Q_tilde * M_inv * C_T)
W_tilde = simplify(B_tilde - Q_tilde * M_inv * (-DM*(qDot) + dVdt))

% Solve for the Lagrange Multipliers
LAMBDA = simplify(([lambda_1; lambda_2] == Q_tilde_new * W_tilde))

% Constraint forces from Lagrangian Multipliers
F_1 = C_T_1 * lambda_1
F_2 = C_T_2 * lambda_2

toc
