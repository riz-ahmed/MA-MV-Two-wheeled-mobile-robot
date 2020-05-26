%  Daniel Kawano, Rose-Hulman Institute of Technology
%  Last modified:  Feb 13, 2016

clear all
close all
clc

%  Specify symbolic parameters and symbolic variables that are functions of
%  time:

syms psi(t) theta(t) phi(t) Dpsi(t) Dtheta(t) Dphi(t) x1(t) x2(t)
syms m g r lambdat lambdaa L R
assume((m > 0) & (g > 0) & (r > 0) & (lambdat > 0) & (lambdaa > 0) & ...
       (L > 0))

%  (1)  Angular velocity kinematics:

%  Relate the space-fixed basis {E1,E2,E3} to the corotational basis
%  {e1,e2,e3} using a 3-1-3 set of Euler angles:

R1 = [cos(psi), sin(psi), 0;        
      -sin(psi), cos(psi), 0;
      0, 0, 1]; 
  
R2 = [1, 0, 0;                    	
      0, cos(theta), sin(theta);
      0, -sin(theta), cos(theta)];

R3 = [cos(phi), sin(phi), 0;        
      -sin(phi), cos(phi), 0;
      0, 0, 1];  

%  Express the angular velocity vector in terms of {e1,e2,e3}:  
  
omega = simplify((R3*R2*R1)*[0; 0; diff(psi)] + ...
                 (R3*R2)*[diff(theta); 0; 0] + R3*[0; 0; diff(phi)]);

%  (2)  Kinematic constraints:

%  For rolling without slipping, the disk's instantaneous point of contact 
%  with the ground has zero velocity.

vCOM = simplify(transpose(R3*R2*R1)*cross(omega, R3*[0; r; 0]));

%  Non-integrable constraints:

Dx1 = [1, 0, 0]*vCOM;
Dx2 = [0, 1, 0]*vCOM;

%  Integrable constraint:

Dx3 = [0, 0, 1]*vCOM;

x3 = simplify(int(Dx3));

%  Simplify the kinematic equations for future manipulation: 

omega = subs(omega, [diff(psi), diff(theta), diff(phi)], [Dpsi, Dtheta, ...
             Dphi]);

Dx1 = subs(Dx1, [diff(psi), diff(theta), diff(phi)], [Dpsi, Dtheta, Dphi]);
Dx2 = subs(Dx2, [diff(psi), diff(theta), diff(phi)], [Dpsi, Dtheta, Dphi]);
Dx3 = subs(Dx3, [diff(psi), diff(theta), diff(phi)], [Dpsi, Dtheta, Dphi]);

%  (3)  Balance of linear momentum:

%  Solve for the normal (constraint) force and the required components of
%  the lateral static friction force to prevent slipping:

F1 = m*diff(Dx1) + R*cos(psi);
F2 = m*diff(Dx2) + R*sin(psi);
N = m*diff(Dx3) + m*g;

%  (4)  Balance of angular momentum with respect to the disk's mass center:

%  Taking the disk to be axisymmetric, evaluate the absolute time 
%  derivative of the angular momentum about the mass center:

H = diag([lambdat, lambdat, lambdaa])*omega;
omegaRF = omega;

DH = diff(H) + cross(omegaRF, H);

%  Sum moments about the mass center:

sumM = cross(R3*[0; -r; 0], (R3*R2*R1)*[F1; F2; N]) + ...
       cross(R3*[0; 0; -L], R3*[-R; 0; 0]);

%  Construct the second-order ODEs for rotational motion of the disk:

ODEsRot = DH == sumM;

%  (5)  Manipulate the system of ODEs into a form suitable for numerical 
%       integration:

%  Express the second-order ODEs in first-order form:

ODEsRot1 = simplify(subs(ODEsRot, [diff(psi), diff(theta), diff(phi)], ...
                         [Dpsi, Dtheta, Dphi]));

%  Manipulating the first two ODEs ultimately yields a cleaner form for the
%  state equations:                       

ODEsRot1 = [sin(phi), -cos(phi), 0;
            cos(phi), sin(phi), 0;
            0, 0, 1]*ODEsRot1;  

%  Relate the state variables in first-order form.  The first-order ODEs
%  associated with the non-integrable constraints are incorporated here:

ODEsRot2Constr = [diff(psi) == Dpsi; diff(theta) == Dtheta; ...
                  diff(phi) == Dphi;  diff(x1) == Dx1; diff(x2) == Dx2];

%  Compile the state equations and arrange the state variables:
     
StateEqns = simplify([ODEsRot1; ODEsRot2Constr]);

StateVars = [Dpsi; Dtheta; Dphi; psi; theta; phi; x1; x2];

%  Express the state equations in mass-matrix form, M(t,Y)*Y'(t) = F(t,Y):

[Msym, Fsym] = massMatrixForm(StateEqns, StateVars);

Msym = simplify(Msym)
Fsym = simplify(Fsym)

%  Convert M(t,Y) and F(t,Y) to symbolic function handles with the input 
%  parameters specified:

M = odeFunction(Msym, StateVars, m, g, r, lambdat, lambdaa, L, R);  
F = odeFunction(Fsym, StateVars, m, g, r, lambdat, lambdaa, L, R);

%  Save M(t,Y) and F(t,Y):

save rolling_disk_ODEs.mat Msym Fsym StateVars M F