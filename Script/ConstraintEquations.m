clear all
clc

% From Paris

syms X Y DX(t) DY(t) DDX(t) DDY(t)
syms theta(t) phi(t) psi(t) Dtheta(t) Dphi(t) Dpsi(t) DDtheta(t) DDphi(t) DDpsi(t)
syms m g R I_a I_t lambda_1 lambda_2
assume((m > 0) & (g > 0) & (R > 0) & (I_a > 0) & (I_t > 0))

% constraint equations
DX = R*cos(phi)*Dpsi - R*sin(theta)*cos(phi)*Dphi - R*cos(theta)*sin(phi)*Dtheta;
DY = R*sin(phi)*Dpsi - R*sin(theta)*sin(phi)*Dphi + R*cos(theta)*cos(phi)*Dtheta;

% Diff constraint equations

DDX = simplify(diff(DX));
DDX = simplify(subs(DDX, [diff(theta) diff(phi) diff(psi) diff(Dtheta) diff(Dphi) diff(Dpsi)], [Dtheta Dphi Dpsi DDtheta DDphi DDpsi]));

DDY = simplify(diff(DX));
DDY = simplify(subs(DDY, [diff(theta) diff(phi) diff(psi) diff(Dtheta) diff(Dphi) diff(Dpsi)], [Dtheta Dphi Dpsi DDtheta DDphi DDpsi]));

% differentials

diff_1 = simplify(diff(I_a*(Dpsi - Dphi*sin(theta))) == -lambda_1*R*cos(phi) - lambda_2*R*sin(phi));
% diff_1 = simplify(subs(diff_1, [diff(theta) diff(Dpsi) diff(Dphi)], [Dtheta DDpsi DDphi]));

DDphi_1 = isolate(diff_1, diff(Dphi));

diff_2 = simplify(diff(-I_a*(Dpsi - Dphi*sin(theta))*sin(theta)+I_t*Dphi*cos(theta).^2) == lambda_1*R*sin(theta)*cos(phi) + lambda_2*R*sin(theta)*sin(phi));
% diff_2 = simplify(subs(diff_2, [diff(Dphi) diff(Dpsi)], [DDphi DDpsi]))
DDpsi_1 = isolate(diff_2, diff(Dpsi));

diff_3 = diff((m*R^2*sin(theta).^2 + I_t)*Dtheta);
t1 = m*R^2*Dtheta^2*sin(theta)*cos(theta);
t2 = I_a*(Dpsi - Dphi*sin(theta))*Dphi*cos(theta);
t3 = I_t*Dphi^2*sin(theta)*cos(theta);
t4 = m*g*R*sin(theta);
diff_31 = simplify(diff_3 -t1 + t2 + t3 -t4 == lambda_1*R*cos(theta)*sin(phi) - lambda_2*R*cos(theta)*cos(phi));
DDtheta_1 = isolate(diff_31, diff(Dtheta));

% listing out the substitutions
% DDX = lambda_1/m;
% DDY = lambda_2/m;
% DDtheta;
% DDpsi;
% DDphi;










