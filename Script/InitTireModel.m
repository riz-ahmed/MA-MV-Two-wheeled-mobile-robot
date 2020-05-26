clear all
clc

%% Parameters
g = 9.81;
% Wheel
radiusRW = 0.3;                                                                             % radius (m)
lengthRW = 0.03;                                                                            % thickness (m)
massRW = 2;                                                                                 % mass (kg)
rhoRW = massRW / (pi * radiusRW^2 * lengthRW);                                              % density (kg/m^3)
momentInertiaRW = [0.0603 0.0603 0.12];                                                     % Moment of inertia (kg-m^2)
% momentInertiaRW = [0.25*massRW*radiusRW^2 0.25*massRW*radiusRW^2 0.5*massRW*radiusRW^2];

% Plane

xPla = 25;                % x plane length (m)
yPla = 8;                 % y plane length (m)
zPla = 0.01;              % z plane depth (m)

%% Tire parameters
% 1. Contact

k_contact_RW = 1e7;                % verical stiffness (N/m)    
b_contact_RW = 2e3;                % damping (N-s/m)

% 2. Stiffness properties

C_F_alpha = 2e4;                   % Lateral Stiffness (N)
C_F_kappa = 2e4;                   % Longitudinl Stiffness (N)
CM_phi = 1e-3;                    % Self-Aligining stiffness

% tire relaxation length
sigma = 0.12;                      % (m)
% propogation speed relaxation
epsilon = 1e-2;                    % Correction factor (not to be modified !!)

%% Simulation parameters

% Joint IC's (Wheel)
phi0 = 10;                          % Intial roll angle (deg)

% IC's on velocity (Wheel)
v0 = 2;                             % Initial velocity (m/s)
Dtheta0 = -v0 / radiusRW;           % Initial angular velocity [spin] (rad/s)
Dphi0 = 0.0872665;                          % Initial angular velocity [roll] (rad/s)
Dpsi0 = 0;                          % Initial angular velocity [yaw] (rad/s)
