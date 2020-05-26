clear all
clc

%% Vehicle Parameters

g = 9.81;

% 1. Shaft
lengthShaft = 1.5;                                                                      % Wheel base betwen the rear and the front axle (m)
radiusShaft = 0.03;                                                                     % Radius of the wheel base(m)

% 2. Rear Wheel (right)
radiusRW = 0.3;                                                                         % Radius of the rear right wheel (m)
lengthRW = 0.03;                                                                        % thickness of the rear right wheel (m)
massRW = 2;                                                                             % Mass of the rear right wheel(kg)
rhoRW = massRW / (pi * radiusRW^2 * lengthRW);                                          % Density (kg/m^3)
momentInertiaRW = [0.25*massRW*radiusRW^2 0.25*massRW*radiusRW^2 0.5*massRW*radiusRW^2];% MI (kg-m^2)

% 2. Rear Wheel (left)
radiusLW = 0.3;                                                                          % Radius of the rear left wheel (m)
lengthLW = 0.03;                                                                         % thickness of the rear left wheel (m)
massLW = 2;                                                                              % Mass of the rear left wheel(kg)
rhoLW = massRW / (pi * radiusLW^2 * lengthLW);                                           % Density (kg/m^3)
momentInertiaLW = [0.25*massLW*radiusLW^2 0.25*massLW*radiusLW^2 0.5*massLW*radiusLW^2]; % MI (kg-m^2)

% 3. Front Wheel
radiusFW = 0.3;                                                                          % Radius of the front wheel (m)
lengthFW = 0.03;                                                                         % thickness of the front wheel (m)
massFW = 2;                                                                              % Mass of the front wheel(kg)
rhoFW = massFW / (pi * radiusFW^2 * lengthFW);                                           % Density (kg/m^3)
momentInertiaFW = [0.25*massFW*radiusFW^2 0.25*massFW*radiusFW^2 0.5*massFW*radiusFW^2]; % MI (kg-m^2)

% 4. Rear Axle
lengthRearAxle = 0.75;                                                                    % Length of the rear axle (m)
radiusRearAxle = 0.02;                                                                    % Radius (m)

%% Plane parameters

xPla = 1000;                % x plane length (m)
yPla = 1000;                % y plane length (m)
zPla = 0.01;              % z plane depth (m)

% Grid parameters
Grid.clr = [1 1 1]*1;
Grid.numSqrs = 500;
Grid.lineWidth = 0.02;
Grid.box_h = (xPla-(Grid.lineWidth*(Grid.numSqrs+1)))/Grid.numSqrs;
Grid.box_l = (xPla-(Grid.lineWidth*(1+1)))/1;
Grid.extr_data = Extr_Data_Mesh(yPla,yPla,Grid.numSqrs,1,Grid.box_h,Grid.box_l);

%% Tire parametrs
% 1.Contact parameters
% (all wheels have same parameters, can be changed)
k_contact_RW = 1e7;             % verical stiffness (N/m)
b_contact_RW = 1e4;             % damping (N-s/m)

% 2. Stiffness properties
% (all wheels have same parameters, can be changed)
C_F_alpha = 1e3;                % Lateral Stiffness (N)
C_F_kappa = 1e3;                % Longitudinl Stiffness (N)
CM_phi = 1e-2;                  % Self-Aligining stiffness

% (all wheels have same parameters, can be changed)
% 3. tire relaxation length
sigma = 0.12;                   % in (m)
% 4. propogation speed relaxation
epsilon = 1e-2;                 % Correction factor (not to be modified !!)

%% Camera parameters

IsoCamX = -1.0;
IsoCamY = 0.75;
IsoCamZ = -2.5;

%% Simulation parameters

% Initial conditions on wheel velocities
% (all wheels are loaded with same conditions, which can also be changed individually)

v0 = 6;                       % Initial forward velocity (m)
Dtheta0 = -v0 / radiusRW;     % Initial angular velocity wheel (m)