clear all
clc
close all

% Euler angle sequence: 3-2-1

% R(psi,theta,phi) = L(psi,p3)L(theta,t2d)L(phi,t1)

syms p1 p2 p3 % world reference frame
syms t1d t2d t3d % first transformed frame
syms t1dd t2dd t3dd % second transformed frame
syms t1 t2 t3 % third transformed frame
syms psi theta phi % transforming Euler angles

% Dot product projections of transformed basis (individual transformations)
% (I) Transformations of individual corotational basis
%     1. L(psi,p3) tranfromes p_i --> tid

t1d_t2d_t3d = [cos(psi) sin(psi) 0;
              -sin(psi) cos(psi) 0;
              0 0 1] * [p1;p2;p3];

% 2. L(psi,p3) tranfromes tid --> tidd

t1dd_t2dd_t3dd = [cos(theta) 0 -sin(theta);
                  0 1 0;
                  sin(theta) 0 cos(theta)] * [t1d;t2d;t3d];

% 3. L(psi,p3) tranfromes tidd --> ti              
t1_t2_t3 = [1 0 0; 
            0 cos(phi) sin(phi);
            0 -sin(phi) cos(phi)] * [t1dd; t2dd; t3dd];
        
% (II)Inverse transformations from corotational basis back to previous basis

% 1. from ti back to tidd
t1dd_t2dd_t3dd = inv([1 0 0; 
                  0 cos(phi) sin(phi);
                  0 -sin(phi) cos(phi)]) * [t1;t2;t3];
              
% 2. from tidd back to tid
t1d_t2d_t3d = inv([cos(theta) 0 -sin(theta);
                  0 1 0;
                  sin(theta) 0 cos(theta)]) * [t1dd; t2dd; t3dd];
              
% from tidd back to original frame pi
p1_p2_p3 = inv([cos(psi) sin(psi) 0;
              -sin(psi) cos(psi) 0;
              0 0 1]) * [t1d;t2d;t3d];
          

% (III) Combined dot product (transforamtions) in all the three individual
% corotational basis to get back to the original basis (p1,p2,p3)
% also called rotational tensor

Rot_tensor = [cos(psi) sin(psi) 0;
              -sin(psi) cos(psi) 0;
              0 0 1] * [cos(theta) 0 -sin(theta);
                  0 1 0;
                  sin(theta) 0 cos(theta)] * [1 0 0; 
            0 cos(phi) sin(phi);
            0 -sin(phi) cos(phi)];
        
 % to get back to basis (p1,p2,p3)
 p1_p2_p3 = Rot_tensor * [t1;t2;t3];
          
% to get to basis (t1,t2,t3)
 t1_t2_t3 = inv(Rot_tensor) * [t1;t2;t3];      
         
    
          