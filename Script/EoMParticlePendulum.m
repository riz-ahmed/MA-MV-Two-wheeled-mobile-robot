clear all
clc
% EoM for a particle pendulum of mass M attched through a massless spring
syms theta(t) u(t)
syms M l K g
syms THETA U Dtheta Du real
assume((M>0) & (l>0) & (K>0) & (g>0))
% theta(t) is angular disp. and u(t) is the spring deflection
% K is the spring constant
% l is the length of the string to which particle is attached

% [1] Generelized coordinates
q = [theta; u];

% [2] Kinetic energy
% [2.1] Linear KE
T_L = 0.5 * M * diff(u,t)^2;
% [2.2] Rotational KE
J = M * (l + u)^2;
T_R = 0.5 * J * diff(theta,t)^2;
% [2.3] Total KE
T = T_R + T_L;

% [3] Potential Energy
% [3.1] PE of the spring
V_S = 0.5 * K * u^2;
% [3.2] PE of the mass
% [3.2.1] undeflected string lenght at equlilibirum
len_UD = l;
% [3.2.2] Projection of length due to swing and spring deflection
len_D = (l+u)*cos(theta);
% [3.2.3] delta_l
delta_l = len_UD - len_D;
% [3.2] ==>
V_M = M * g * (l - (l + u)*cos(theta));
% [3.3] Total PE
V = V_M + V_S;

% [4] Lagrange's EoM (Unconstrained motion)
% [4.1] EoM1
ActList = [theta(t) u(t) diff(theta,t) diff(u,t)];
HolList = [THETA U Dtheta Du];
T = subs(T,ActList,HolList);
V = subs(V,ActList,HolList);
delT_THETA = diff(T,THETA);
delV_THETA = diff(V,THETA);
delT_Dtheta = diff(T,Dtheta);
delT_Dtheta = subs(delT_Dtheta, HolList, ActList);
dT_Dtheta = diff(delT_Dtheta);
delV_THETA = subs(delV_THETA,HolList, ActList);
delT_THETA = subs(delT_THETA, HolList, ActList);
EoM1 = (simplify(dT_Dtheta - delT_THETA + delV_THETA) == 0);

% [4.2] EoM2
ActList = [theta(t) u(t) diff(theta,t) diff(u,t)];
HolList = [THETA U Dtheta Du];
T = subs(T,ActList,HolList);
V = subs(V,ActList,HolList);
delT_U = diff(T,U);
delV_U = diff(V,U);
delT_Du = diff(T,Du);
delT_Du = subs(delT_Du, HolList, ActList);
dT_Du = diff(delT_Du);
delV_U = subs(delV_U,HolList, ActList);
delT_U = subs(delT_U, HolList, ActList);
EoM2 = (simplify(dT_Du - delT_U + delV_U) == 0);