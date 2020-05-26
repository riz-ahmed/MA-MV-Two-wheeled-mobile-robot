clc
syms m g J real
syms x(t) y(t) psi(t)
syms X Y PSI Dx Dy Dpsi DDx DDy DDpsi real

ActualList = [x(t) y(t) psi(t) diff(x) diff(y) diff(psi) diff(x,t,t) diff(y,t,t) diff(psi,t,t)];
HolderList = [X Y PSI Dx Dy Dpsi DDx DDy DDpsi];

% mass matrix
M = [m 0 0;
     0 m 0;
     0 0 J];
 
% Constraint coefficient matrix
A = [-sin(psi) cos(psi) 0];
b = diff(x)*cos(psi) + diff(y)*sin(psi);

% Velocity terms
Q = 0;
 
% Solve for lambdae
lambda = simplify(-inv(A*inv(M)*transpose(A)) * (b - A*inv(M)*Q));
 
% Solve for accelerations
DDq = simplify(inv(M)*Q - inv(M)*transpose(A) * lambda);

% Constraint forces
Qr = -transpose(A)*lambda;

% Print
Lambda = subs(lambda,ActualList, HolderList)
DDq = subs(DDq,ActualList, HolderList)
Qr_1 = [1 0 0] * subs(Qr,ActualList,HolderList)
Qr_2 = [0 1 0] * subs(Qr,ActualList, HolderList)