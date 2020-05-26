syms psi(t) theta(t) phi(t) d_psi(t) d_theta(t) d_phi(t) x1(t) x2(t)
syms m g r lambda_t lambda_a L R

f_sys = [lambda_t*sin(theta) 0 0 0 0 0 0 0;
        0 2*m*r^2+2*lambda_t 0 0 0 0 0 0;
        cos(theta)*(m*r^2+lambda_a) 0 m*r^2+lambda_a 0 0 0 0 0;
        0 0 0 1 0 0 0 0;
        0 0 0 0 1 0 0 0;
        0 0 0 0 0 1 0 0;
        0 0 0 0 0 0 1 0;
        0 0 0 0 0 0 0 1];
    
Msys = [L*R + lambda_a*d_phi*d_theta + lambda_a*cos(theta)*d_psi*d_theta - 2*lambda_t*cos(theta)*d_psi*d_theta;
        lambda_t*d_psi^2*sin(2*theta) - lambda_a*d_psi^2*sin(2*theta) - m*r^2*d_psi^2*sin(2*theta) - 2*lambda_a*sin(theta)*d_phi*d_psi - 2*g*m*r*cos(theta) - 2*m*r^2*sin(theta)*d_phi*d_psi;
        r*(R + 2*m*r*sin(theta)*d_psi*d_theta) + lambda_a*sin(theta)*d_psi*d_theta;
        d_psi;
        d_theta;
        d_phi;
        r*sin(psi)*sin(theta)*d_theta - r*cos(psi)*cos(theta)*d_psi - r*cos(psi)*d_phi;
        -r*(sin(psi)*d_phi + cos(theta)*sin(psi)*d_psi + cos(psi)*sin(theta)*d_theta);
 ];

clear all; close all; clc

load rolling_disk_ODEs

% parameters

r = 0.5;                % radius of the disk (m)
m = 5;                  % mass of the disk (kg)
g = 9.81;               % gravity (m/s^2)
lambda_t = 1/4*m*r^2;   % MI about laong and lat directions
lambda_a = 1/2*m*r^2;   % MI about verticle direction
L = 0.1;                % disk length (m)
R = 1;                  % external force (N)

% simulation parameters
t0 = 0;                 % start time (s)
tEnd = 10;              % end time (s)
N = 0.1;                % interval (s)
tSim = [t0: N: tEnd];   % simulation time

% initializing

psidot0 = 0;                    %  rad/s
thetadot0 = 0;                  %  rad/s
phidot0 = pi;                   %  rad/s
psi0 = 0*(pi/180);              %  rad
theta0 = 90*(pi/180);           %  rad
phi0 = 0*(pi/180);              %  rad
x10 = 0;                        %  m
x20 = 0;                        %  m

% initial condition

Y0 = [psidot0; thetadot0; phidot0; psi0; theta0; phi0; x10; x20];

% ODEs

M = @(t,Y) M(t,Y,m, g, r, lambdat, lambdaa, L, R);
F = @(t, Y) F(t, Y, m, g, r, lambdat, lambdaa, L, R);

[t,Y] = ode45(F, tSim, Y0);

%  Extract the results:

psidot = Y(:,1);            %  rad/s
thetadot = Y(:,2);          %  rad/s
phidot = Y(:,3);            %  rad/s
psi = Y(:,4);               %  rad
theta = Y(:,5);             %  rad
phi = Y(:,6);               %  rad
x1 = Y(:,7);                %  m
x2 = Y(:,8);                %  m

% calculate the contact point position

x3 = r*sin(theta);          % (m)

%  Calculate the normal force and the tangential forces

x1dot = -r*cos(psi).*cos(theta).*psidot + ...                   	%  m/s
        r*sin(psi).*sin(theta).*thetadot - r*cos(psi).*phidot;         
x2dot = -r*sin(psi).*cos(theta).*psidot - ...                    	%  m/s
        r*cos(psi).*sin(theta).*thetadot - r*sin(psi).*phidot;        
x3dot = r*cos(theta).*thetadot;                                   	%  m/s

x1ddot = gradient(x1dot,dt);            %  m/s^2
x2ddot = gradient(x2dot,dt);            %  m/s^2
x3ddot = gradient(x3dot,dt);            %  m/s^2

F1 = m*x1ddot + R*cos(psi);             %  N
F2 = m*x2ddot + R*sin(psi);             %  N
N = m*x3ddot + m*g;                     %  N
