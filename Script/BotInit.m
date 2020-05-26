% Initializing libraries
HomeDir = pwd;
addpath(genpath(pwd));
parts_libname = 'Parts_Lib';
phy_lib = 'Contact_Forces_Lib';
load_system(parts_libname);
load_system(phy_lib);

% square
a_S = 0.33;
b_S = a_S*2;
rho_S = 500;
l_S = 0.1/2;
area_S = a_S^2;
mass_S = (area_S*l_S)*rho_S;
cm_S = [a_S/2 a_S/2];

% trapeziod
a_t = 0.1;
b_t = 0.33*2;
h_t = 0.6 - 0.33;
rho_t = rho_S;
area_t = ((a_t + b_t)/2)*h_t;
mass_t =  (area_t * l_S) * rho_t;

% base geometry
geo_base = [0 0; a_S/2 0; a_S/2 a_S; a_t/2 (a_S+h_t); -a_t/2 (a_S+h_t); -a_S/2 a_S; -a_S/2 0];
mass_base = mass_S + mass_t;

% base parameters
c_t = (b_t - a_t)/2;
x1 = (2*a_t*c_t + c_t*b_t + a_t*b_t + b_t^2) / (3*(a_t+b_t));
x2 = (h_t*(2*a_t + b_t)) / (3*(a_t + b_t));
cm_t = [x1 x2];
cm_base_x = (mass_S*cm_S(1) + mass_t*cm_t(1))/(mass_S+mass_t);
cm_base_y = (mass_S*cm_S(2) + mass_t*cm_t(2))/(mass_S+mass_t);

% axle
axle_dia = l_S/2;
axle_length = a_S+0.1;

% wheel_R and wheel_L
radius_W = 0.0950;
length_W = 0.03;
rho_W = 000;
mass_w = (pi*radius_W^2*length_W)*rho_W;

% sphere
radius_sph = 0.05/2;
rho_sph = 000;
mass_sph = ((4/3)*pi*radius_sph^3)*rho_sph;

% vehcile mass
mass_bot = mass_base + 2*mass_w + mass_sph;

% plane
xPla = 2;
yPla = 10;
zPla = 0.01;

% constants
g = 9.81;