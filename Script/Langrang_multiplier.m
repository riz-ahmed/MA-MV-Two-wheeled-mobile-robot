syms r psi d_psi d_theta phi d_phi lambda    % declrating the variables

% oject funtion (equations of motion)
f1 = r*(cos(psi))*d_theta - r*(sin(phi)*cos(psi))*d_psi - r*(cos(phi)*sin(psi))*d_phi;
f2 = r*(sin(psi))*d_theta - r*(sin(phi)*sin(psi))*d_psi + r*(cos(phi)*cos(psi))*d_psi;

% first constrait equation
g1 = r*d_theta*cos(psi);

% second constrait equation
g2 = r*d_theta*sin(psi);

% Lagrange equation
L1 = f1 + lambda*g1;
L2 = f2 + lambda*g2;

% solving the gradients (L1)
L1_r = simplify(diff(L1,r));
L1_psi = simplify(diff(L1,psi));
L1_d_psi = simplify(diff(L1,d_psi));
L1_d_theta = simplify(diff(L1,d_theta));
L1_phi = simplify(diff(L1,phi));
L1_d_phi = simplify(diff(L1,d_phi));

pretty(f1)
pretty(g1)
L1m = [L1_r; L1_psi; L1_d_psi; L1_d_theta; L1_phi; L1_d_phi];
L1m

% solving the gradients (L2)
L2_r = simplify(diff(L1,r));
L2_psi = simplify(diff(L1,psi));
L2_d_psi = simplify(diff(L1,d_psi));
L2_d_theta = simplify(diff(L1,d_theta));
L2_phi = simplify(diff(L1,phi));
L2_d_phi = simplify(diff(L1,d_phi));

pretty(f2)
pretty(g1)
L2m = [L1_r; L1_psi; L1_d_psi; L1_d_theta; L1_phi; L1_d_phi];
L1m