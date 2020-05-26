clear all
clc
close all

% Intializing variables in symbolic form as function of time

syms X_dot(t) Y_dot(t) theta(t) phi(t) psi(t) theta_dot(t) phi_dot(t) psi_dot(t)
syms r lambda

% Equations of motion
X_dot = r*(cos(theta))*theta_dot - r*(sin(phi)*cos(psi))*psi_dot - r*(cos(phi)*sin(psi))*phi_dot;
Y_dot = r*(sin(phi))*theta_dot - r*(sin(phi)*sin(psi))*psi_dot + r*(cos(phi)*cos(psi))*phi_dot;

% Constraints
x_dot = r*theta_dot*cos(psi);
y_dot = r*theta_dot*sin(psi);

% Lagrange formulation
LX_dot = X_dot - lambda * (x_dot);
LY_dot = Y_dot - lambda * (y_dot);

% Gradients
L1_dr = diff(LX_dot,r);
L1_psi = diff(LX_dot,psi);
L1_theta = diff(LX_dot,theta);
L1_phi = diff(LX_dot,phi);
L1_Dpsi = diff(LX_dot,psi_dot);
L1_Dtheta = diff(LX_dot,theta_dot);
L1_Dphi = diff(LX_dot,phi_dot);