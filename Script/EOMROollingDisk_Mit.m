clear all
clc

syms psi(t) theta(t) phi(t) X(t) Y(t)
syms m g Jt Ja R lambda_1 lambda_2
assume((m > 0) & (g > 0) & (R > 0) & (Jt > 0) & (Ja > 0))

%  Angular velocity kinematics:

%  Relate the space-fixed basis {I1,I2,I3} to the corotational basis
%  {B1,B2,B3} using a 3-1-3 set of Euler angles:

R1 = [cos(psi), sin(psi), 0;        
      -sin(psi), cos(psi), 0;
      0, 0, 1]; 
  
R2 = [1, 0, 0;                    	
      0, cos(theta), sin(theta);
      0, -sin(theta), cos(theta)];

R3 = [cos(phi), sin(phi), 0;        
      -sin(phi), cos(phi), 0;
      0, 0, 1];  

%  Express the angular velocity vector in terms of {B1,B2,B3}:  
  
omega = simplify((R3*R2*R1)*[0; 0; diff(psi)] + ...
                 (R3*R2)*[diff(theta); 0; 0] + R3*[0; 0; diff(phi)]);


% Angular velocities given in {B1,B2,B3}:

omega_1 = diff(theta,t)*cos(phi) + diff(psi,t)*(sin(phi)*sin(theta));
omega_2 = diff(psi,t)*(cos(phi)*sin(theta))-diff(theta,t)*sin(phi);
omega_3 = diff(phi,t) + diff(psi,t)*cos(theta);

% Kinetic Energy
vSquared = (diff(X,t))^2 + (diff(Y,t))^2 + R^2*sin(theta).^2*(diff(theta,t))^2;
T = 1/2*m*vSquared + Ja/2*(omega_3)^2 + Ja/2*(omega_1^2 + omega_2^2);

% Potential Energy
V = m*g*R*cos(theta);

% Lagrangian
L = T - V;

% L_new
syms PSI THETA PHI THE_X THE_Y Dpsi Dtheta Dphi DX DY
ActualList = [psi theta phi X Y diff(psi,t) diff(theta,t) diff(phi,t) diff(X,t) diff(Y,t)];
HolderList = [PSI THETA PHI THE_X THE_Y Dpsi Dtheta Dphi DX DY];

L_new = subs(L,ActualList,HolderList);

% gradient in X
dL_dq_X = diff(L_new,THE_X);
dL_dqDot_X = diff(L_new,DX);
dL_dqDot_X = subs(dL_dqDot_X, HolderList, ActualList);
dL_dqDot_X = diff(dL_dqDot_X,t);
EOM_1 = (dL_dqDot_X - dL_dq_X == lambda_1);

% gradient in Y
dL_dq_Y = diff(L_new,THE_Y);
dL_dqDot_Y = diff(L_new,DY);
dL_dqDot_Y = subs(dL_dqDot_Y, HolderList, ActualList);
dL_dqDot_Y = diff(dL_dqDot_Y,t);
EOM_2 = (dL_dqDot_Y - dL_dq_Y == lambda_2);

% gradient in psi
dL_dq_psi = diff(L_new,PSI);
dL_dqDot_psi = diff(L_new,Dpsi);
dL_dqDot_psi = subs(dL_dqDot_psi, HolderList, ActualList);
dL_dqDot_psi = diff(dL_dqDot_psi,t);
EOM_3 = (dL_dqDot_psi - dL_dq_psi == 0);

% gradient in theta
dL_dq_theta = diff(L_new,THETA);
dL_dqDot_theta = diff(L_new,Dtheta);
dL_dqDot_theta = subs(dL_dqDot_theta, HolderList, ActualList);
dL_dqDot_theta = diff(dL_dqDot_theta,t);
EOM_4 = (dL_dqDot_theta - dL_dq_theta == 0);

% gradient in phi
dL_dq_phi = diff(L_new,PHI);
dL_dqDot_phi = diff(L_new,Dphi);
dL_dqDot_phi = subs(dL_dqDot_phi, HolderList, ActualList);
dL_dqDot_phi = diff(dL_dqDot_phi,t);
EOM_5 = (dL_dqDot_phi - dL_dq_phi == -lambda_1*R*cos(phi)-lambda_2*R*sin(phi));




