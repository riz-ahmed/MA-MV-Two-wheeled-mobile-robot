% Deriving velocity constraint equations for a disk rolling on a plane with
% no-slips and expressing the constraints w.r.t the disk's mass center
% in kinematic referene frame F

clear all
clc

syms X(t) Y(t) Z(t) phi(t) psi(t) theta(t)
syms DX DY DZ Dpsi Dphi Dtheta PSI THETA PHI
syms R
assume(R>0)

%  (1) Angular velocity kinematics:

%  Relate the space-fixed basis {F1,F2,F3} to the corotational basis
%  {B1,B2,B3} using a 1-2-3 set of angles:

% Frames:
% F : Kinematic reference frame
% F1 : Intermediate frame
% F2 : Body non rotating frame
% B : Body fixed frame

R1 = [1 0 0;                        % R from F to F1
      0 cos(phi) -sin(phi);
      0 sin(phi) cos(phi)]; 
  
R2 = [cos(psi) 0 sin(psi);          % R from F1 to F2
      0 1 0;
      -sin(psi) 0 cos(psi)];

R3 = [cos(theta) -sin(theta) 0;     % R from F2 to B
      sin(theta) cos(theta) 0;
      0 0 1];  

%  Express the angular velocity vector in terms of {F1,F2,F3}:  
  
omega = simplify((R1*R2*R3)*[0;0;diff(theta)] + ...
                (R1*R2)*[0;diff(psi);0] + R1*[diff(phi);0;0]);

omega_F2 = simplify(transpose(R1*R2) * omega);
% (2) Lienar velocity kinematics
%     velocity of disk's center of mass expressed in frame F
v_CM = [diff(X);diff(Y);diff(Z)];
v_CM_F2 = simplify(transpose(R1*R2) * v_CM);
% Calculating velocity of contact point using relative velocity formula for
% rigid body

% Expressing contact point velocity in frame F
v_CP_F2 = simplify(v_CM_F2 + cross(omega_F2,[0;R;0]));           % is it necessary to consider omega?
v_CP_F = simplify((R1*R2*R3)*v_CP_F2);
% v_CP_F = simplify(v_CM + cross(((R1*R2)*[0;diff(psi);0] + R1*[diff(phi);0;0]),(R1*R2)*[0;R;0]));

v_CP_F = subs(v_CP_F,[diff(X),diff(Y),diff(Z),psi(t), phi(t),theta(t), diff(psi), diff(phi), diff(theta)],[DX DY DZ PSI PHI THETA Dpsi Dphi Dtheta]);

% (3) Applying no-slip constraints as vCP == 0 in fram F
[dx,dy,dz] = solve([1,0,0]*v_CP_F==0, [0,1,0]*v_CP_F==0,[0,0,1]*v_CP_F==0, DX, DY, DZ);

% Derived NonHolonomic constrains on velocity near the contact point
disp('Constraint equation near the contact point : ')
dx = simplify(dx)
dy = simplify(dy)
dz = simplify(dz)
disp('Constraint equation near the contact point expressed in frame F2: ')
v_CP_F2 = simplify(transpose(R1*R2) * [dx;dy;dz]);
% [dx1,dy1,dz1] = simplify(([1,0,0]*v_CP_F2,[0,1,0]*v_CP_F2,[0,0,1]*v_CP_F2));
dx1 = simplify([1,0,0]*v_CP_F2)
dy1 = simplify([0,1,0]*v_CP_F2)
dz1 = simplify([0,0,1]*v_CP_F2)
% % Derived NonHolonomic constrains on velocity near disk's mass center
% disp('Constraint equation near the disks mass center : ')
% dx
% dy



