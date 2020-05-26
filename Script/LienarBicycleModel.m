%% Paramters

% general bike
w = 1-02;           % wheel base (m)
c = 0.08;           % trail (m)
lambda = 90 - 72;   % steer axis titlt angle (deg)

% constants
g = 9.81;           % gravity (m/s^2)

% rear wheel
r_R = 0.3;          % rearwheel radius (m)
m_R = 2;            % mass_R (kg)
I_R_xx = 0.0603;    % MI about lat axle axis (long motion direction) (kg-m^2)
I_R_yy = 0.12;      % MI about the axle axis (is greater than about the lat axis (obviously)) (kg-m^2)

% rear frame
x_B = 0.3;          % pos about CP (m)
y_B = 0.9;          % pos about CP (m)
m_B = 85;           % mass of rear frame (kg)
MI_B = [9.2 0 2.4; 0 11 0; 2.4 0 2.8];  % mass momoment of inertia tensor

% front frame
x_H = 0.9;          % pos about CP (m)
y_H = 0.7;          % pos about CP (m)
m_H = 4;            % mass of front frame (kg)  
MI_H = [0.05892 0 -0.00756; 0 0.06 0; -0.00756 0 0.00708];  % mass momoment of inertia tensor

% front wheel
r_F = 0.35;          % rearwheel radius (m)
m_F = 3;            % mass_R (kg)
I_F_xx = 0.1405;    % MI about lat axle axis (long motion direction) (kg-m^2)
I_F_yy = 0.28;      % MI about the axle axis (is greater than about the lat axis (obviously)) (kg-m^2)

%% System dynamics

% Equation of motion is divided into two parts
% 1. for dof : theeta_R
% 2. for combined dof's : phi and delta
% -------------------------------------------
% 1. first part of the equation of motion
% -------------------------------------------
% Traanslational velocity
% with known angular velocity of the rear wheel, the bike forward velocity
% can be determined:

v = - dot_theeta_R * r_R;   

% equation of motion from the following assumptions:
% 1. system is laterally symmetric
% 2. mathematically linear
% concludes:
%--> first-order decoupling between foward motion and the lean and steer

% Resulting linearzined equations of motion:

m_T = m_R + m_B + m_H + m_F;    % total bike mass

combined_mass_MI = ((m_T * r_R^2) + (I_R_yy) + ((r_R/r_F)^2*I_F_yy));
ang_acc = ddot_theeta_R;

% equation of motion for the dof : theeta_R
T_theeta_R = combined_mass_MI * ang_acc;    % actuating torque at the rear axle

% -------------------------------------------
% 2. second part of the equation of motion
% -------------------------------------------

% co-efficients of the linearized equations of the motion

% properties of the rear assymbly
% m_T = m_T;                  % total system mass
x_T = (x_B*m_B + x_H*m_H + w*m_F) / m_T; % pos about CP of the whole system (m)
z_T = (-r_R*m_R + z_B*m_B + z_H*m_H - r_F*m_F) / m_T;   % pos about CP of the whole system (m)

I_T_xx = I_R_xx + I_B_xx + I_H_xx + I_F_xx + m_R*r_R^2 + m_B*z_B^2 + m_H*z_H^2 + m_F*r_F^2; % total system mass MI about long vector direction
I_T_xz = I_B_xy + I_H_xz - m_B*x_B*z_B - m_H*x_H*z_H + m_F*w*r_F;

I_R_zz = I_R_xx;    % dependent MI for axisymetric rear and front wheel
I_F_zz = I_F_xx;

I_T_zz = I_R_zz + I_B_zz + I_H_zz + I_F_zz + m_B*x_B^2 + m_H*x_H^2 + m_F*w^2;   % MI of the system along z-axis

% properties of the front assymbly

m_A = 







