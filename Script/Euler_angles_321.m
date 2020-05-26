clear all
clc
close all

% Deriving equations of motion for a disk rolling on a flate plane without
% slipping

% constraints to apply: Non-holonomic contact point material velocity contraint

syms psi(t) theta(t) phi(t) Dpsi(t) Dtheta(t) Dphi(t)  % psi: orientation, phi: roll theta: spin
syms I1 I2 I3 Z1 Z2 Z3 Bd1 Bd2 Bd3 B1 B2 B3
syms m r g
assume((m > 0) & (g > 0) & (r > 0))

% 3-2-1 Euler angle sequence

R1 = [cos(phi) sin(phi) 0;
            -sin(phi) cos(phi) 0;
            0 0 1];
% Z1_Z2_Z3 = R1 * [I1;I2;I3];

R2 = [cos(theta) 0 -sin(theta);
                0 1 0;
                sin(theta) 0 cos(theta)];
        
% Bd1_Bd2_Bd3 =  R2* [Z1_Z2_Z3(1); Z1_Z2_Z3(2); Z1_Z2_Z3(3)];
  
R3 = [1 0 0;
            0 cos(psi) sin(psi);
            0 -sin(psi) cos(psi)];
% B1_B2_B3 =  R3* [Bd1_Bd2_Bd3(1); Bd1_Bd2_Bd3(2); Bd1_Bd2_Bd3(3)];
        
% R(psi,phi,theta) = L(psi,I3)L(phi,Z2)L(theta,Bd1)     % Bd1 = B1

% Calculating Euler basis
%L(psi,I3)
% I1_I2_I3 = simplify(inv(R1)*inv(R2)*inv(R3)*[B1;B2;B3]);
% I3 = I1_I2_I3(3)
% 
% % L(phi,Z2)
% Z1_Z2_Z3 = simplify(inv(R2)*inv(R3)*[B1;B2;B3]);
% Z2 = Z1_Z2_Z3(2)
% 
% % L(theta,Bd1)
% Bd1_Bd2_Bd3 = simplify(inv(R3)* [B1;B2;B3]);
% Bd1 = Bd1_Bd2_Bd3(1)

% angular velocity
% omega = simplify((diff(psi)*I3) + (diff(phi)*Z2) + (diff(theta)*Bd1))

omega = simplify((R3*R2*R1)*[0; 0; diff(psi)] + ...
                 (R3*R2)*[0; diff(psi); 0] + R3*[diff(theta); 0; 0])





