clear all
clc
%% Bike Parameters

% 1. Base parameters
w = 1.02;                  % wheel base (m)
c = 0.09;                  % trail (m)
lambda = 18;               % steering head angle (deg)

g = 9.81;                  % gravity (m/s^2)
vec_n = [0;1;0];           % plane normal

% 2. Rear Wheel
radiusRW = 0.3;                                                                                         % radius (m)
lengthRW = 0.03;                                                                                        % thickness (m)
massRW = 2;                                                                                             % mass(kg)
rhoRW = massRW / (pi * radiusRW^2 * lengthRW);                                                          % density (kg/m^3)
% momentInertiaRW = [0.0603 0 0.12];
momentInertiaRW = [0.25*massRW*radiusRW^2 0.25*massRW*radiusRW^2 0.5*massRW*radiusRW^2];                % Moment of inertia (kg-m^2)

% 3. Rear Frame
cylRF = 0.05;                       % radius (m)
mB = 85;                            % mass (kg)

% 4. Front Frame                   
cylFF = 0.05;                       % radius (m)
mH = 5;                             % mass (kg)

% 5. Front wheeel
radiusFW = 0.35;                                                                                        % radius (m)
lengthFW = 0.03;                                                                                        % thickness (m)
massFW = 4;                                                                                             % mass (kg)
rhoFW = massFW / (pi * radiusFW^2 * lengthFW);
% momentInertiaFW = [0.1405 0.1405 0.28];
momentInertiaFW = [0.25*massFW*radiusFW^2 0.25*massFW*radiusFW^2 0.5*massFW*radiusFW^2];                % Moment of inertia (kg-m^2)

% 5. Derived geometrical parameters of frames
% 5.1 Base geomtery (not to be modified!!)
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
momentInertiaB = [9.2 11 2.8];                  % Rear Frame Moment of inertia (kg-m^2)
ProductsofInertiaB = [2.4 0 0];                 % Rear Frame products of inertia (kg-m^2)

momentInertiaH = [0.05892 0.06 0.00708];        % Front frame Moment of inertia (kg-m^2)
ProductsofInertiaH = [-0.00756 0 0];            % Front frame products of inertia (kg-m^2)

CenterOfMassB_in_I = [0.3;0;0.9];               % Input Center of Mass here in global reference frame(Rear Frame)
% (Not to be modified!!)
CenterOfMassB_in_Z = [0;(-radiusRW + CenterOfMassB_in_I(3));CenterOfMassB_in_I(1)];
transMx = [1 0 0; 0 cosd(-angleRF) -sind(-angleRF); 0 sind(-angleRF) cosd(-angleRF)];
CenterOfMassB_in_B = transpose(transMx) * CenterOfMassB_in_Z;
d_Z1_in_I = transpose(transMx) * [0;lengthRF/2*sind(angleRF);lengthRF/2*cosd(angleRF)];
CenterOfMassB_in_B = CenterOfMassB_in_B - d_Z1_in_I;
% (------------------------------------------------------------------------------------)

CenterOfMassH_in_I = [0.95;0;0.7];               % Input Center of Mass here in global reference frame(Front Frame)
% (Not to be modified!!)
CenterOfMassH_in_Z = [CenterOfMassH_in_I(1)-w; 0 ; CenterOfMassH_in_I(3)-radiusFW];
transMy = [cosd(-angleFF) 0 sind(-angleFF); 0 1 0; -sind(-angleFF) 0 cosd(-angleFF)];
d_Z2_in_I = transpose(transMy) * [-lengthFF/2*sind(angleFF);0;lengthFF/2*cosd(angleFF)];
CenterOfMassH_in_B = transpose(transMy) * CenterOfMassH_in_Z;
CenterOfMassH_in_B = CenterOfMassH_in_B - d_Z2_in_I;
% (------------------------------------------------------------------------------------)
%% Plane parameters

xPla = 1000;                % x plane length (m)
yPla = 1000;                % y plane length (m)
zPla = 0.01;              % z plane depth (m)

% Grid parameters
Grid.clr = [1 1 1]*1;
Grid.numSqrs = 150;
Grid.lineWidth = 0.02;
Grid.box_h = (xPla-(Grid.lineWidth*(Grid.numSqrs+1)))/Grid.numSqrs;
Grid.box_l = (xPla-(Grid.lineWidth*(1+1)))/1;
Grid.extr_data = Extr_Data_Mesh(yPla,yPla,Grid.numSqrs,1,Grid.box_h,Grid.box_l);
%% Tire parameters

% 1. Contact parameters

% 1. Rear Wheel
k_contact_RW = 1e5;         % Vertical stiffness (N/m)
b_contact_RW = 2e4;         % Damping (N-s/m)

% 2. Front wheel
k_contact_FW = 1e5;        % Vertical stiffness (N/m)
b_contact_FW = 2e4;        % Damping (N-s/m)

% 2. Stiffness properties

C_F_alpha = 2e4;          % Longitudinal stiffness
C_F_kappa = 2e4;          % Lateral stiffness
CM_phi = 1e-1;            % Self-Aligining stiffness

% 3. tire relaxation length
sigma = 0.12;             % (m)
% 4. propogation speed relaxation
epsilon = 1e-2;           % correction factor (Not to be modified!!)

%% Camera Setup
frontCameraX = -1.75;
frontCameraZ = 0.35;

followerCameraX = 7.5;
followerCameraZ = 3;

isoCameraX = -1.1;
isoCameraY = 0.5;
isoCameraZ = -2*w;
isoCameraAimX = (1/2)*w + 0.75;

IsoCameraDamping = 30;
IsoCameraStiffness = 5;
Filtering1Dto3DConnection = 0.1;

topViewY = 4.5*w;
topViewZ = -0.5*w;

%% Simulation parameters

% Joint IC's (Rear Wheel)
phi0 = 0;                       % deg

% IC's on velocity (Rear Wheel)
v0 = 7.3;                         % (m/s)
Dtheta0 = -v0 / radiusRW;       % (rad/s)
Dphi0 = 0;                      % (rad/s)
Dpsi0 = 0;                      % (rad/s)

% IC's on velocity (Front Wheel)
v0_FW = v0;                     % (m/s)
Dtheta0_FW = -v0 / radiusFW;    % (rad/s)

% IC steering angle
delta0 = 0;                     % deg
