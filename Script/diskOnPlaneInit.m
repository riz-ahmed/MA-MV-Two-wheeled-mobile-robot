clear all
clc

%% Initialization file for pureRollingWheelOnPlane
%% parameters
% 
% HomeDir = pwd;
% addpath(genpath(pwd));
% parts_libname = 'Parts_Lib';
% phy_lib = 'Multibody_Multiphysics_Lib';
% load_system(parts_libname);
% load_system(phy_lib);

% Wheel
rRim = 0.5;
radius=rRim;
lRim = 0.05;
rhoDisk = 5400;
mRim = pi*rRim^2*lRim*rhoDisk;

% Tire
s = 0.05;
rTire = rRim+s;
aTire = [rRim lRim/2; rRim -lRim/2; rRim+s -lRim/2; rRim+s lRim/2];
rhoTire = 1500;
a1 = pi*rTire^2;
a0 = pi*rRim^2;
mTire = (a1-a0)*lRim*rhoTire;
      
% plot(aTire)

%% Constants
% gravity
g = 9.81;

%% Plane

% plane
xPlaLgt = 20;
yPlaLgt = 2.5;
zPlaLgt = 0.01;
pDepth = zPlaLgt/2;

% position

delta = 1;
delta2 = 0.3;
sPosX = -xPlaLgt/2 + delta;
sPosY = 0;

%% Contact forces

% Normal force
cgap = zPlaLgt/2+rTire;
k_contact = 1e5;
b_contact =0.3;
vthr = 1e-4;
mustat = 0.6;
Fz = (mRim+mTire)*g;

%% Simulation parameters

vInit = 3;
dAngle = 0.08;