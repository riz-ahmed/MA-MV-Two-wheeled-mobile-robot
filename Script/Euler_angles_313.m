clear all
clc
close all

syms psi(t) phi(t) theta(t) Dpsi(t) Dphi(t) Dtheta(t)             % psi: orientation, theta: roll phi: spin
syms I1 I2 I3 Z1 Z2 Z3 Bd1 Bd2 Bd3 B1 B2 B3
syms r g

% 3-1-3 Euler angle sequence

R1 = [cos(phi), sin(phi), 0;        
      -sin(phi), cos(phi), 0;
      0, 0, 1]; 
  
R2 = [1, 0, 0;                    	
      0, cos(theta), sin(theta);
      0, -sin(theta), cos(theta)];

R3 = [cos(psi), sin(psi), 0;        
      -sin(psi), cos(psi), 0;
      0, 0, 1];  
        
% R(psi,phi,theta) = L(psi,I3)L(theta,Z1)L(phi,Bd3)     % Bd1 = B1

% Calculating Euler basis
%L(psi,I3)
% I1_I2_I3 = simplify(transpose(R1)*transpose(R2)*transpose(R3)*[B1;B2;B3]);
% I3 = I1_I2_I3(3)
% 
% % L(phi,Z2)
% Z1_Z2_Z3 = simplify(transpose(R2)*transpose(R3)*[B1;B2;B3]);
% Z1 = Z1_Z2_Z3(1)
% 
% % L(theta,Bd1)
% Bd1_Bd2_Bd3 = simplify(transpose(R3)* [B1;B2;B3]);
% Bd3 = Bd1_Bd2_Bd3(3)

% angular velocity

omega = simplify((R3*R2*R1)*[0; 0; diff(psi)] + ...
                 (R3*R2)*[diff(theta); 0; 0] + R3*[0; 0; diff(phi)])



