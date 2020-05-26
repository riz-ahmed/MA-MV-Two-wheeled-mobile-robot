% Reference: [A bicycle model for education in multibody dynamics and real-time interactive simulation
%                   Escalona J Recuero A ]
clear all
clc
syms x_c(t) y_c(t) z_c(t) phi(t) theta(t) psi(t) beta(t) gamma(t) epsilon(t)
syms R
syms Dx_c Dy_c Dz_c Dphi Dtheta Dpsi Dbeta Dgamma Depsilon real
syms X_C Y_C Phi Theta Psi Beta Gamma Epsilon real
assume(R>0)

% Kinematics
% 1. Transfomrmations
R_I = [cos(phi) -sin(phi) 0;
        sin(phi) cos(phi) 0;
        0 0 1];
R_i1 = [1 0 0;
        0 cos(theta) -sin(theta);
        0 sin(theta) cos(theta)];
    
% 2. Velocity kinematics
% (i) Linear Velocity
r_G2_i2 = [0;0;R];
T = R_I * R_i1;
r_c = [x_c; y_c; 0];
r_G2 = r_c + T * r_G2_i2

v_G2 = diff(r_G2)


ActualList = [x_c(t) y_c(t) phi(t) theta(t) psi(t) beta(t) gamma(t) epsilon(t) diff(x_c) diff(y_c) diff(phi) diff(theta) diff(psi) diff(beta) diff(gamma) diff(epsilon)];
HolderList = [X_C Y_C Phi Theta Psi Beta Gamma Epsilon Dx_c Dy_c Dphi Dtheta Dpsi Dbeta Dgamma Depsilon];
v_G2 = subs(v_G2,ActualList,HolderList);

% Generelized coordinates
q = [x_c(t) y_c(t) phi(t) theta(t) psi(t) beta(t) gamma(t) epsilon(t)];
qH = [X_C Y_C Phi Theta Psi Beta Gamma Epsilon];

% Jacobian of the CM position wrt q
H = jacobian(v_G2,[Dx_c Dy_c Dphi Dtheta Dpsi Dbeta Dgamma Depsilon])

% (