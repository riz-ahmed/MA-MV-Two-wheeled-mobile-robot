clc
close all
%% Wheel
% subplot(3,1,1)
% plot(Fx.TIME, Fx.Data)
% title('Longitudinal Force in N')
% ylabel('Fx')
% ylim([-110 60])
% 
% subplot(3,1,2)
% plot(Fy.TIME, Fy.Data)
% title('Normal Force in N')
% ylabel('Fy')
% ylim([-110 60])

% subplot(3,1,3)
% plot(Fz.TIME, Fz.Data)
% xlabel('time in s')
% title('Lateral Force in N')
% ylabel('Fz')
% ylim([-110 60])

% figure;
hold on
plot(xAxis,zAxis,'LineWidth',1.5)
xlabel('x-axis (m)','FontSize',14)
ylabel('z-axis (m)','FontSize',14)
xlim([-1 20])
plot(xAxis(1),zAxis(1),'o','MarkerFaceColor','red')
txt = '  \leftarrow Starting Point';
text(xAxis(1),zAxis(1),txt,'FontSize',14)
plot(xAxis(end),zAxis(end),'o','MarkerFaceColor','red')
txt = '  \leftarrow End Point';
text(xAxis(end),zAxis(end),txt,'FontSize',14)
title('Tricycle trajectory (m)','FontSize',14)
% 
% figure;
% plot(alpha_ip.TIME,alpha_ip.Data)
% xlabel('time in s')
% ylabel('\alpha')
% title('Input slip angle')

% figure;
% plot(alpha_F.TIME,alpha_F.Data)
% xlabel('time in s')
% ylabel('\alpha_F')
% title('Output slip response')

% Tricycle
figure;

subplot(2,1,1)
plot(Obs_pos.TIME,Obs_pos.Data,'LineWidth',1.5)
xlabel('time (s)','FontSize',14)
ylabel('Angular position (deg) of observer','FontSize',14)

subplot(2,1,2)
plot(Obs_Felt_Moment.TIME,Obs_Felt_Moment.Data,'LineWidth',1.5)
xlabel('time (s)','FontSize',14)
ylabel('Felt torque (N-m) by an observer','FontSize',14)

figure;

subplot(2,1,1)
plot(SteeringTorque.TIME,SteeringTorque.Data,'LineWidth',1.5)
xlabel('time (s)','FontSize',14)
ylabel('Input steering torque (N/m)','FontSize',14)

subplot(2,1,2)
plot(SteerAngle.TIME,SteerAngle.Data,'LineWidth',1.5)
xlabel('time (s)','FontSize',14)
ylabel('Output steer angle (deg)','FontSize',14)