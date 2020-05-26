clear all
clc
%
% psi - 3 - orientataion
% phi - 2 - rolling
% theta - 1 - spin

syms x(t) y(t) psi(t) phi(t) theta(t)
syms X Y PSI PHI THETA DX DY DPSI DPHI DTHETA DDX DDY DDPSI DDPHI ...
     DDTHETA real
syms m R J real
syms Fx Fy Tx Ty Tz

% Lists
ActualList = [x(t) y(t) psi(t) phi(t) theta(t) diff(x) diff(y) diff(psi) diff(phi) diff(theta) ...
              diff(x,t,t) diff(y,t,t) diff(psi,t,t) diff(phi,t,t) diff(theta,t,t)];
HolderList = [X Y PSI PHI THETA DX DY DPSI DPHI DTHETA DDX DDY DDPSI DDPHI DDTHETA];

% 1. Angular velocity kinematics
R1 = [cos(psi) -sin(psi) 0;
      sin(psi) cos(psi) 0;
      0 0 1];
  
R2 = [cos(phi) 0 sin(phi);
      0 1 0;
      -sin(phi) 0 cos(phi)];
  
R3 = [1 0 0;
      0 cos(theta) -sin(theta);
      0 sin(theta) cos(theta)];
  
% omega expressed in base frame
omega = simplify((R1*R2*R3)*[diff(theta);0;0] + ...
                  (R1*R2)*[0;diff(phi);0] + R1*[0;0;diff(psi)]);
              
% omega in B frame
omega_B = simplify(transpose(R1*R2*R3)*omega);

omega_B = subs(omega_B,ActualList, HolderList);

%--------------------------------------------------------------------------
% 2. Totatl kinetic energy
%--------------------------------------------------------------------------
onega_B = subs(omega_B,HolderList, ActualList);
T = simplify(0.5 * m * (diff(x)^2 + diff(y)^2) + 0.5 * J * (([1 0 0]*(omega_B))^2 + ([0 1 0]*omega_B)^2 ...
                + ([0 0 1]*omega_B)^2));

T = subs(T,ActualList,HolderList);
[Coeff,Tc] = coeffs(T,[DX,DY,DPSI,DPHI,DTHETA]);
            
% 3. Potention Energy
V = 0;

% 4. External forces and torques (resolved in B frame)
% Q_ext = [Fx;Fy;0;Mx;My;Mz];

%--------------------------------------------------------------------------
% 5. EoM for an unconstrained motion
%--------------------------------------------------------------------------

% q1 = x
delT_delq1dot = diff(T,DX);
delT_delq1 = diff(T,X);
delV_delq1 = diff(V,X);
% time differential term
delT_delq1dot = subs(delT_delq1dot,HolderList,ActualList);
d_dt_q1 = diff(delT_delq1dot);
d_dt_q1 = subs(d_dt_q1,ActualList,HolderList);
% EoM_1
EoM1 = simplify(d_dt_q1 - delT_delq1 + delV_delq1 == Fx)

%q2 = y
delT_delq2dot = diff(T,DY);
delT_delq2 = diff(T,Y);
delV_delq2 = diff(V,Y);
% time differential term
delT_delq2dot = subs(delT_delq2dot,HolderList,ActualList);
d_dt_q2 = diff(delT_delq2dot);
d_dt_q2 = subs(d_dt_q2,ActualList,HolderList);
EoM2 = simplify(d_dt_q2 - delT_delq2 + delV_delq2 == Fy)

%q3 = psi
delT_delq3dot = diff(T,DPSI);
delT_delq3 = diff(T,PSI);
delV_delq3 = diff(V,PSI);
% time differential term
delT_delq3dot = subs(delT_delq3dot,HolderList,ActualList);
d_dt_q3 = diff(delT_delq3dot);
d_dt_q3 = subs(d_dt_q3,ActualList,HolderList);
EoM3 = simplify(d_dt_q3 - delT_delq3 + delV_delq3 == Tz)


% q4 = theta
delT_delq4dot = diff(T,DPHI);
delT_delq4 = diff(T,PHI);
delV_delq4 = diff(V,PHI);
% time differential term
delT_delq4dot = subs(delT_delq4dot,HolderList,ActualList);
d_dt_q4 = diff(delT_delq4dot);
d_dt_q4 = subs(d_dt_q4,ActualList,HolderList);
EoM4 = simplify(d_dt_q4 - delT_delq4 + delV_delq4 == Ty)

% q5 = phi
delT_delq5dot = diff(T,DTHETA);
delT_delq5 = diff(T,THETA);
delV_delq5 = diff(V,THETA);
% time differential term
delT_delq5dot = subs(delT_delq5dot,HolderList,ActualList);
d_dt_q5 = diff(delT_delq5dot);
d_dt_q5 = subs(d_dt_q5,ActualList,HolderList);
EoM5 = simplify(d_dt_q5 - delT_delq5 + delV_delq5 == Tx)










            