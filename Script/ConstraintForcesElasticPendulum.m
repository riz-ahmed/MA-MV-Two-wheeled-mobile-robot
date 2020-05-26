clear all
clc
% Solving for equations of motions and forces applied due to constraints
% in a holonomic elastic pendulum

syms theta(t) u(t)
syms m l k g
syms THETA U Dtheta Du real
assume((m>0) & (l>0) & (k>0) & (g>0))
% theta(t) is angular disp. and u(t) is the spring deflection
% K is the spring constant
% l is the length of the string to which particle is attached

% [1] Generelized coordinates
q = [theta; u];

% [2] Kinetic energy
% [2.1] Linear KE
T_L = 0.5 * m * diff(u,t)^2;
% [2.2] Rotational KE
J = m * (l + u)^2;
T_R = 0.5 * J * diff(theta,t)^2;
% [2.3] Total KE
T = T_R + T_L;
% [2.4] KE in mass matrix form
[Coeff_qDot c1] = coeffs(T,[diff(theta,t) diff(u,t)]);
m11 = Coeff_qDot * transpose([1 0]);
m12 = 0;
m21 = 0;
m22 = Coeff_qDot * transpose([0 1]);
M = [2*m11 m12; m21 2*m22];
disp('Total KE : ')
T = simplify(0.5 * transpose(diff(q)) * M * diff(q))

% [3] Potential Energy
% [3.1] PE of the spring
V_S = 0.5 * k * u^2;
% [3.2] PE of the mass
% [3.2.1] undeflected string lenght at equlilibirum
len_UD = l;
% [3.2.2] Projection of length due to swing and spring deflection
len_D = (l+u)*cos(theta);
% [3.2.3] delta_l
delta_l = len_UD - len_D;
% [3.2] ==>
V_M = m * g * (delta_l);
% [3.3] Total PE
disp('Total KE : ')
V = simplify(V_M + V_S)

% (4) EoM
% (4.1) Lagrangian


















