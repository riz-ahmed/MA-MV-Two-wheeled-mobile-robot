clear all
clc
%% Initializing libraries

%% Bike Parameters

% 1. Base parameters
w = 1.02;                  % wheel base
c = 0.09;
lambda = 18;    

g = 9.81;
vec_n = [0;1;0];           % plane normal

% 2. Rear Wheel
radiusRW = 0.3;             % (m)
lengthRW = 0.03;
massRW = 2;                     % (kg)
rhoRW = massRW / (pi * radiusRW^2 * lengthRW);
% momentInertiaRW = [0.0603 0 0.12];
momentInertiaRW = [0.25*massRW*radiusRW^2 0.25*massRW*radiusRW^2 0.5*massRW*radiusRW^2];

% 3. Rear Frame
cmRF_X = 0.3;           % from rear wheel CP (m)
cmRF_Y = 0.9;           % from rear wheel CP (m)

cylRF = 0.05; 
mB = 85;

% 4. Front Frame
cmFF_X = 0.9;
cmFF_Y = 0.7;
cylFF = 0.05;
mH = 5;

% 5. Front wheeel
radiusFW = 0.35;
lengthFW = 0.03;
massFW = 4;
rhoFW = massFW / (pi * radiusFW^2 * lengthFW);
% momentInertiaFW = [0.1405 0.1405 0.28];
momentInertiaFW = [0.25*massFW*radiusFW^2 0.25*massFW*radiusFW^2 0.5*massFW*radiusFW^2];

% 5. Derived geometrical parameters of frames
% 5.1 Base geomtery
frontHinge = 0.15 * w;
lengthFF = frontHinge / (sind(lambda));
adj = lengthFF * cosd(lambda);
lengthRF = sqrt((w-frontHinge)^2 + adj^2);
angleRF =  atan(adj / (w-frontHinge));
angleRF = angleRF * (180/pi);
steeringAngle = (90 - lambda);
angleFF = (lambda);
% angleFF = angleFF * (pi/180);

% 6. Moment of Inertias of rear and front frame
momentInertiaB = [9.2 11 2.8];                  % Rear Frame
ProductsofInertiaB = [2.4 0 0];

momentInertiaH = [0.05892 0.06 0.00708];        % Front frame
ProductsofInertiaH = [-0.00756 0 0];

CenterOfMassB_in_I = [0.3;0;0.9];               % Input CM here (Rear Frame)
CenterOfMassB_in_Z = [0;(-radiusRW + CenterOfMassB_in_I(3));CenterOfMassB_in_I(1)];
transMx = [1 0 0; 0 cosd(-angleRF) -sind(-angleRF); 0 sind(-angleRF) cosd(-angleRF)];
CenterOfMassB_in_B = transpose(transMx) * CenterOfMassB_in_Z;
d_Z1_in_I = transpose(transMx) * [0;lengthRF/2*sind(angleRF);lengthRF/2*cosd(angleRF)];
CenterOfMassB_in_B = CenterOfMassB_in_B - d_Z1_in_I;

CenterOfMassH_in_I = [0.95;0;0.7];                        % Input CM here (Front frame)
CenterOfMassH_in_Z = [CenterOfMassH_in_I(1)-w; 0 ; CenterOfMassH_in_I(3)-radiusFW];
transMy = [cosd(-angleFF) 0 sind(-angleFF); 0 1 0; -sind(-angleFF) 0 cosd(-angleFF)];
d_Z2_in_I = transpose(transMy) * [-lengthFF/2*sind(angleFF);0;lengthFF/2*cosd(angleFF)];
CenterOfMassH_in_B = transpose(transMy) * CenterOfMassH_in_Z;
CenterOfMassH_in_B = CenterOfMassH_in_B - d_Z2_in_I;
%% Plane parameters

xPla = 25;                % x plane length (m)
yPla = 15;                % y plane length (m)
zPla = 0.01;              % z plane length (m)

%% Contact parameters

k_contact_RW = 1e6;
b_contact_RW = 2e3;

k_contact_FW = 1e6;
b_contact_FW = 2e3;

%% Stiffness properties

C_F_alpha = 2e4;          % Longitudinal stiffness
C_F_kappa = 2e4;          % Lateral stiffness
CM_phi = 1e-1;            % 

%% Tire dynamics

% tire relaxation length
sigma = 0.12;
% propogation speed relaxation
epsilon = 1e-2;

%% Simulation parameters

% Joint IC's (Rear Wheel)
phi0 = 0;          % deg

% IC's on velocity (Rear Wheel)
v0 = 7.3;
Dtheta0 = -v0 / radiusRW;
Dphi0 = 0;
Dpsi0 = 0;

% IC's on velocity (Front Wheel)
v0_FW = v0;
Dtheta0_FW = -v0 / radiusFW;

% IC steering angle
delta0 = 0;             % deg
