%  Daniel Kawano, Rose-Hulman Institute of Technology
%  Last modified:  Feb 13, 2016

clear all
close all
clc

%  Load the symbolic function handles for the state equations in
%  mass-matrix form, M(t,Y)*Y'(t) = F(t,Y):

load rolling_disk_ODEs

%  Physical parameters:

m = 5;                          %  kg
g = 9.81;                       %  m/s^2
r = 0.5;                        %  m
lambdat = 1/4*m*r^2;            %  kg-m^2
lambdaa = 1/2*m*r^2;            %  kg-m^2
L = 0.1;                        %  m
R = 1;                          %  N

%  Simulation parameters:

dt = 0.01;                      %  s
tf = 14;                        %  s
tsim = [0 : dt : tf]';          %  s    
tol = 1e-6;

psidot0 = 0;                    %  rad/s
thetadot0 = 0;                  %  rad/s
phidot0 = pi;                   %  rad/s
psi0 = 0*(pi/180);              %  rad
theta0 = 90*(pi/180);           %  rad
phi0 = 0*(pi/180);              %  rad
x10 = 0;                        %  m
x20 = 0;                        %  m

Y0 = [psidot0, thetadot0, phidot0, psi0, theta0, phi0, x10, x20]';

%  Plotting parameters:

span = [0.8, 1.2];

%  Initial conditions for steady motion of the rolling disk.  Specify the
%  rate of precession and the nutation angle to determine the required spin
%  rate:

% R = 0;      %  To be extra careful, set the applied force R = 0 here.
% 
% psidot0 = -0.15*(2*pi);         %  rad/s
% theta0 = 70*(pi/180);           %  rad
% 
% phidot0 = ((lambdat - lambdaa - m*r^2)*sin(theta0)* ...     %  rad/s
%            psidot0^2 - m*g*r)/((lambdaa + m*r^2)* ...
%                                tan(theta0)*psidot0);
% 
% Y0 = [psidot0, 0, phidot0, 0, theta0, 0, 0, 0]';

%  Convert M(t,Y) and F(t,Y) to purely numeric function handles for
%  numerical integration:

M = @(t, Y) M(t, Y, m, g, r, lambdat, lambdaa, L, R);
F = @(t, Y) F(t, Y, m, g, r, lambdat, lambdaa, L, R);

%  Numerically integrate the state equations:

options = odeset('mass', M, 'abstol', tol, 'reltol', tol);

[t, Y] = ode45(F, tsim, Y0, options);

%  Extract the results:

psidot = Y(:,1);            %  rad/s
thetadot = Y(:,2);          %  rad/s
phidot = Y(:,3);            %  rad/s
psi = Y(:,4);               %  rad
theta = Y(:,5);             %  rad
phi = Y(:,6);               %  rad
x1 = Y(:,7);                %  m
x2 = Y(:,8);                %  m

%  Plot the precession rate, nutation angle, and spin rate over time:

figure
set(gcf, 'color', 'w')
subplot(311)
plot(t, psidot/(2*pi), '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('Precession rate (rev/s)')
ylim([min(min(psidot/(2*pi))*span), max(max(psidot/(2*pi))*span)])
subplot(312)
plot(t, theta*(180/pi), '-r', 'linewidth', 2)
xlabel('Time (s)')
ylabel('Nutation angle (deg)')
ylim([min(min(theta*(180/pi))*span), max(max(theta*(180/pi))*span)])
subplot(313)
plot(t, phidot/(2*pi), '-k', 'linewidth', 2)
xlabel('Time (s)')
ylabel('Spin rate (rev/s)')
ylim([min(min(phidot/(2*pi))*span), max(max(phidot/(2*pi))*span)])

%  Calculate the vertical position of the disk's mass center, and then plot
%  the mass center location over time:

x3 = r*sin(theta);           %  m

figure
set(gcf, 'color', 'w')
subplot(311)
plot(t, x1, '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_{1} (m)')
subplot(312)
plot(t, x2, '-r', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_{2} (m)')
subplot(313)
plot(t, x3, '-k', 'linewidth', 2)
xlabel('Time (s)')
ylabel('\itx\rm_{3} (m)')
ylim([0, 2*r])

%  Calculate the normal force and the magnitude of the friction force over 
%  time to check for slipping:

x1dot = -r*cos(psi).*cos(theta).*psidot + ...                   	%  m/s
        r*sin(psi).*sin(theta).*thetadot - r*cos(psi).*phidot;         
x2dot = -r*sin(psi).*cos(theta).*psidot - ...                    	%  m/s
        r*cos(psi).*sin(theta).*thetadot - r*sin(psi).*phidot;        
x3dot = r*cos(theta).*thetadot;                                   	%  m/s

x1ddot = gradient(x1dot,dt);            %  m/s^2
x2ddot = gradient(x2dot,dt);            %  m/s^2
x3ddot = gradient(x3dot,dt);            %  m/s^2

F1 = m*x1ddot + R*cos(psi);             %  N
F2 = m*x2ddot + R*sin(psi);             %  N
Ff = sqrt(F1.^2 + F2.^2);               %  N
N = m*x3ddot + m*g;                     %  N

%  Plot the minimum coefficient of static friction over time needed to 
%  prevent slipping:

figure
set(gcf, 'color', 'w')
plot(t, Ff./N, '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('Coefficient of static friction')
ylim([min(min(Ff./N)*span), max(max(Ff./N)*span)])

%  Calculate and plot the disk's total mechanical energy over time:

omega1 = psidot.*sin(theta).*sin(phi) + thetadot.*cos(phi);     %  rad/s
omega2 = psidot.*sin(theta).*cos(phi) - thetadot.*sin(phi);     %  rad/s
omega3 = psidot.*cos(theta) + phidot;                           %  rad/s

E = 1/2*m*(x1dot.^2 + x2dot.^2 + x3dot.^2) + ...            %  J
    1/2*(lambdat*omega1.^2 + lambdat*omega2.^2 + ...
         lambdaa*omega3.^2) + m*g*x3;

figure
set(gcf, 'color', 'w')
plot(t, E, '-b', 'linewidth', 2)
xlabel('Time (s)')
ylabel('Total mechanical energy (J)')
ylim([min(min(E)*span), max(max(E)*span)])
        
%  Animate the motion of the disk:

animate_rolling_disk(r, psi, theta, phi, x1, x2, x3, dt);