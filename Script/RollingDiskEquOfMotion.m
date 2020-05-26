clear all
clc
close all

% Deriving equations of motion for a rolling disk
% Euler angle sequence 3-1-3 (psi,theta,phi)

syms psi theta phi
syms dpsi dtheta dphi
syms p3 t1d t3
syms t1 t2 t3

% parameters
% psi = (30*pi)/180;
% theta = (60*pi)/180;
% phi = (30*pi)/180;
% t1 = 1; t2 =1; t3 = 1;
% Rotation matrix

R1 = [cos(psi), sin(psi), 0;        
      -sin(psi), cos(psi), 0;
      0, 0, 1]; 
  
R2 = [1, 0, 0;                    	
      0, cos(theta), sin(theta);
      0, -sin(theta), cos(theta)];

R3 = [cos(phi), sin(phi), 0;        
      -sin(phi), cos(phi), 0;
      0, 0, 1];  

% Rotation tensor

Rot_tensor = R1*R2*R3;
            
%
p30 = Rot_tensor(3,:)*[t1;t2;t3];
            

%
p3 = sin(phi)*sin(theta)*t1 + cos(phi)*sin(theta)*t2 + cos(theta)*t3;
t1d = cos(phi)*t1 + (-sin(phi)*t2) + 0;
t3 = 0+0+t3;

%
omega = simplify(diff(psi)*p3 + diff(theta)*t1d + diff(t3)*t3);
            
% omega
omega1 = (diff(psi)*sin(phi)*sin(theta) + diff(theta)*cos(phi))*t1 + ...
        (diff(psi)*cos(phi)*sin(theta) - diff(theta)*sin(phi))*t2 + ...
        (diff(psi)*cos(theta) + diff(phi))*t3;
    
omega2 = simplify((R3*R2*R1)*[0; 0; diff(psi)] + ...
                 (R3*R2)*[diff(theta); 0; 0] + R3*[0; 0; diff(phi)]);