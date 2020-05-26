% Finding Euler angles from known transofrmation matrix R

syms psi phi theta real
syms r m g real
syms T real                     % Transfomration matrix

psi = atan2(-T(2,3),-T(1,3));
phi = atan2(sqrt((T(1,3))^2 + (T(2,3))^2),T(3,3));
theta = atan2(-T(3,2),T(3,1));

