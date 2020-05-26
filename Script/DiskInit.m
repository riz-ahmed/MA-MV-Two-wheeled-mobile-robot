%% Initializing libraries

HomeDir = pwd;
addpath(genpath(pwd));
parts_libname = 'Parts_Lib';
phy_lib = 'Contact_Forces_Lib';
load_system(parts_libname);
load_system(phy_lib);

%% Constants

g = 9.81;               % (m/s^2)


%% Plane parameters

xPla = 15;                % x plane length (m)
yPla = 5;                 % y plane length (m)
zPla = 0.01;              % z plane length (m)

%% Disk Parameters

radiusRW = 0.3;
lengthRW = 0.03;
rhoRW = 1000;

%% Contact parameters

d_beta = 0;                   % angle of road inclination about the plane perpendicular to body fixed wheel symmetry plane
beta = d_beta * pi / 180;
road_normal = [sin(beta) cos(beta) 0];
Kpen = 100000;
Dpen = 1000;
C_F_kappa = 1000;
C_F_alpha = 1000;