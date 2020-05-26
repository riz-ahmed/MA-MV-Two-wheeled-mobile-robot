clear all
clc
close all

syms psi(t) theta(t) phi(t) Dpsi(t) Dtheta(t) Dphi(t) DDotpsi(t) DDotphi(t) DDottheta(t)
syms X(t) Y(t) dX(t) dY(t) ddotX(t) ddotY(t)
syms R m g J_a J_t lambda_1 lambda_2 LAMBDA_1 LAMBDA_2
 
DDotpsi = (J_a*sin(theta(t))*diff(Dphi(t), t) + J_t*cos(theta(t))^2*diff(Dphi(t), t) + J_a*cos(theta(t))*Dphi(t)*diff(theta(t), t) - R*lambda_1*cos(phi(t))*sin(theta(t)) - R*lambda_2*sin(phi(t))*sin(theta(t)) - 2*J_t*cos(theta(t))*sin(theta(t))*Dphi(t)*diff(theta(t), t))/J_a;

DDotphi = -(diff(Dpsi(t), t) - cos(theta(t))*Dphi(t)*diff(theta(t), t))/sin(theta(t));
 
DDottheta = -(J_a*(Dpsi(t) - (Dphi(t)^2*sin(2*theta(t)))/2) + (J_t*Dphi(t)^2*sin(2*theta(t)))/2 + (R^2*m*Dtheta(t)^2*sin(2*theta(t)))/2 + R*lambda_2*cos(phi(t))*cos(theta(t)) - R*lambda_1*cos(theta(t))*sin(phi(t)) - R*g*m*sin(theta(t)))/(m*R^2*sin(theta(t))^2 + J_t);

DDotX = R*cos(phi(t))*diff(Dpsi(t), t) - R*sin(phi(t))*Dpsi(t)*diff(phi(t), t) - R*cos(theta(t))^2*Dphi(t)*diff(theta(t), t) + R*sin(theta(t))^2*Dphi(t)*diff(theta(t), t) - R*cos(theta(t))*sin(phi(t))*diff(Dtheta(t), t) - R*cos(theta(t))*sin(theta(t))*diff(Dphi(t), t) - R*cos(phi(t))*cos(theta(t))*Dtheta(t)*diff(phi(t), t) + R*sin(phi(t))*sin(theta(t))*Dtheta(t)*diff(theta(t), t);
sDDotX = simplify(subs(DDotX, [diff(Dpsi(t), t) diff(Dtheta(t), t) diff(Dphi(t), t)], [DDotpsi DDottheta DDotphi]));
LAMBDA_1 = simplify(sDDotX * m);

DDotY = simplify(diff(R*sin(phi)*Dpsi) - R*(sin(theta)*sin(phi))*Dphi - R*(cos(theta)*cos(phi))*Dtheta);
sDDotY = simplify(subs(DDotY, [diff(Dpsi(t), t) diff(Dtheta(t), t) diff(Dphi(t), t) lambda_2], [DDotpsi DDottheta DDotphi m*ddotY]));
LAMBDA_2 = simplify(sDDotY * m);
simplify(LAMBDA_2 - m*ddotY)