clear all
clc

syms m l g K real
syms theta u Dtheta Du DDtheta DDu real

% M
M = [m*(l+u)^2 0;
     0 m];
 
% Q
Q = [-m*g*(l+u)*sin(theta)-2*m*(l+u)*Du*Dtheta;
     m*(l+u)*Dtheta^2 - K*u + m*g*cos(theta)];
 
 % A
 A = [2*theta 1/l];
 
% b
b = [-2*Dtheta^2];

% lambda
lambda = simplify(-inv(A*inv(M)*transpose(A)) * (b - A*inv(M)*Q))
 
% Qr
 Qr = simplify(-transpose(A) * lambda);
 Qr1 = simplify([1 0] * Qr)
 Qr2 = simplify([0 1] * Qr)
 
 % Parameters
 m = 1;
 g = 9.81;
 l = 1;
 K = 25.6;
 