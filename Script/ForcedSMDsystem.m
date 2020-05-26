% Forced spring mass damper system
syms u(t) f(t)      % one q(t)
% f(t) is the external force
% damping force c*u_dot is a function of delta_u and goes into the terms
% of generelized forces of the system
syms M K c U F Du Df real

% KE
T = 0.5 * M * (diff(u,t))^2;
% PE
V = 0.5 * K * u^2;
% Damper does not store the energy, but it can exert force of on the system
% the force is a real non-conservative force derived from the principle of
% virtual work

% p (sum of NC forces and external forces acting on the system)
NC_forces = - c * diff(u,t);
Ext_forces = f(t);
p = NC_forces + Ext_forces;

ActList = [u f diff(u,t) diff(f,t)];
HolList = [U F Du Df];
T = subs(T,ActList,HolList);
V = subs(V, ActList, HolList);

% EoM
% LHS
dT_Du = diff(T,Du);
dT_u = diff(T,U);
dV_u = diff(V,U);
dT_Du = subs(dT_Du, HolList, ActList);
dT_u = subs(dT_u,HolList,ActList);
dV_u = subs(dV_u,HolList,ActList);
d_dt = diff(dT_Du);
LHS = d_dt - dT_u + dV_u;

% RHS (NC forces)
% delta_W = F * delta_q/dt :== 0
% Q_r = F
Q_r = NC_forces + f(t); % RHS

EoM = (LHS == Q_r)





