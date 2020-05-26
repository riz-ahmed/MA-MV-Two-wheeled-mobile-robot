clear all
clc

syms m J real
syms X Y PSI DX DY DPSI DDX DDY DDPSI
syms x(t) y(t) psi(t)

syms Force(t) Torque(t)                    % External forces abd torques

% Lists
ActualList = [x y psi diff(x) diff(y) diff(psi) diff(x,t,t) diff(y,t,t) diff(psi,t,t)];
HolderLsit = [X Y PSI DX DY DPSI DDX DDY DDPSI];

% Generelized coordinates

q = [x; y; psi];

% Kinemtic Energy
T = 0.5 * m * (diff(x)^2 + diff(y)^2) + 0.5 * J * diff(psi)^2;

T = subs(T,ActualList, HolderLsit);

[Coeff,Tc] = coeffs(T,[DX,DY,DPSI]);

% Eq_1 : Mass matrix
M = [Coeff*[1;0;0] 0 0;
     0 Coeff*[0;1;0] 0;
     0 0 Coeff*[0;0;1]];
 
% Potential energy
V = 0;

% Nonholonomic constraints
g = diff(y)*cos(psi) - diff(x)*sin(psi);
g = subs(g,ActualList,HolderLsit);
[gCoeff,gc] = coeffs(g,[DX,DY]);

% Eq_2 Constrain coefficent matrix
A = [gCoeff*[1; 0] gCoeff*[0; 1] 0];

g = subs(g,HolderLsit,ActualList);
gDot = diff(g);
gDot = subs(gDot,ActualList,HolderLsit);
[gDotCoeff,gDotc] = coeffs(gDot,[DDX DDY DDPSI]);

% Eq_3 Constrain b matrix
b = [gDotCoeff * [0;0;1] * -1];

% EoM for an unconstrianed system
% q1 = x
delT_delq1dot = diff(T,DX);
delT_delq1 = diff(T,X);
delV_delq1 = diff(V,X);
% time differential term
delT_delq1dot = subs(delT_delq1dot,HolderLsit,ActualList);
d_dt_q1 = diff(delT_delq1dot);
d_dt_q1 = subs(d_dt_q1,ActualList,HolderLsit);
EoM1 = simplify(d_dt_q1 - delT_delq1 + delV_delq1 == Force);

% q2 = y
delT_delq2dot = diff(T,DY);
delT_delq2 = diff(T,Y);
delV_delq2 = diff(V,Y);
% time differential term
delT_delq2dot = subs(delT_delq2dot,HolderLsit,ActualList);
d_dt_q2 = diff(delT_delq2dot);
d_dt_q2 = subs(d_dt_q2,ActualList,HolderLsit);
EoM2 = simplify(d_dt_q2 - delT_delq2 + delV_delq2 == 0);

% q3 = psi
delT_delq3dot = diff(T,DPSI);
delT_delq3 = diff(T,PSI);
delV_delq3 = diff(V,PSI);
% time differential term
delT_delq3dot = subs(delT_delq3dot,HolderLsit,ActualList);
d_dt_q3 = diff(delT_delq3dot);
d_dt_q3 = subs(d_dt_q3,ActualList,HolderLsit);
EoM3 = simplify(d_dt_q3 - delT_delq3 + delV_delq3 == Torque);

[EoMCoeff1,EoMc1] = coeffs(EoM1,[DDX,DDY,DDPSI]);
[EoMCoeff2,EoMc2] = coeffs(EoM2,[DDX,DDY,DDPSI]);
[EoMCoeff3,EoMc3] = coeffs(EoM3,[DDX,DDY,DDPSI]);

% E1_4 Non constraiend Q matrix
Q = [Force;
    0;
    Torque];
 
 
% lambda
Lambda = -inv(A*inv(M)*transpose(A))*(b-A*inv(M)*Q)

% Constrain forces
Qr_1 = [1 0 0] * -transpose(A)*Lambda
Qr_2 = [0 1 0] * -transpose(A)*Lambda

% Parameters
m = 0.2;
% wedge
h = 10;
b = 5;
a = b/2;
Ixx = (b*h^3)/12;
Iyy = (h*b^3 + h*a*b^2 + h*a^2*b) / 12;
J = (h*b^3 + h*a*b^2 + h*a^2*b + b*h^3) / 12;
Geo = [b/2 0; 0 h; -b/2 0];
cm = [(Geo(1,1)+Geo(2,1)+Geo(3,1))/3 , (Geo(1,2)+Geo(2,2)+Geo(3,2))/3];
% plane
xPla = 100;
yPla = 100;
zPla = 0.01;


