%  Daniel Kawano, Rose-Hulman Institute of Technology
%  Last modified:  Feb 13, 2016

function animate_rolling_disk(r, psi, theta, phi, x1, x2, x3, dt)

%  Create a circle for the disk:

angle = linspace(0, 2*pi, 40)';
circ1 = r*cos(angle);
circ2 = r*sin(angle);

%  The plane of the disk varies over time according to the evolution of
%  the corotational basis {e1,e2,e3}.  Solve for how {e1,e2,e3} vary over
%  time.  Also, keep track of the disk's instantaneous point of contact P
%  with the ground and a material point A on the disk's periphery:

for k = 1:length(psi)
    R1 = [cos(psi(k)), sin(psi(k)), 0;              %  3-1-3 set of               
          -sin(psi(k)), cos(psi(k)), 0;             %  Euler angles
          0, 0, 1];
    R2 = [1, 0, 0;                               
          0, cos(theta(k)), sin(theta(k));       
          0, -sin(theta(k)), cos(theta(k))];    
    R3 = [cos(phi(k)), sin(phi(k)), 0;           
          -sin(phi(k)), cos(phi(k)), 0;          
          0, 0, 1];
    e2pp(:,k) = ([0, 1, 0]*(R2*R1))';                       %  e2''
    e1(:,k) = ([1, 0, 0]*(R3*R2*R1))';                      %  e1
    e2(:,k) = ([0, 1, 0]*(R3*R2*R1))';                      %  e2        
    e3(:,k) = ([0, 0, 1]*(R3*R2*R1))';                      %  e3
    xcirc(:,k) = x1(k) + circ1*e1(1,k) + circ2*e2(1,k);     %  m
    ycirc(:,k) = x2(k) + circ1*e1(2,k) + circ2*e2(2,k);     %  m
    zcirc(:,k) = x3(k) + circ1*e1(3,k) + circ2*e2(3,k);     %  m
    xP(k,1) = x1(k) - r*e2pp(1,k);                          %  m
    yP(k,1) = x2(k) - r*e2pp(2,k);                          %  m
    zP(k,1) = 0;                                            %  m
    xA(k,1) = x1(k) + r*e2(1,k);                            %  m
    yA(k,1) = x2(k) + r*e2(2,k);                            %  m
    zA(k,1) = x3(k) + r*e2(3,k);                            %  m
end

%  Set up the figure window:

figure
set(gcf, 'color', 'w')
plot3(x1(1), x2(1), x3(1));
xlabel('\itx\rm_{1} (m)')
set(gca, 'xdir', 'reverse')
ylabel('\itx\rm_{2} (m)')
set(gca, 'ydir', 'reverse')
zlabel('\itx\rm_{3} (m)            ', 'rotation', 0)
axis equal
xlim([min(x1)-r, max(x1)+r])
ylim([min(x2)-r, max(x2)+r])
zlim([0, 1.2*(2*r)])
grid on

%  Trace out the path of the instantaneous contact point P, and orient the 
%  disk appropriately.  Also, highlight a material point A on the disk's 
%  periphery:

path = line('xdata', xP(1:1), 'ydata', yP(1:1), 'zdata', zP(1:1), 'color', 'b', 'linewidth', 2);
disk = patch('xdata', xcirc(:,1), 'ydata', ycirc(:,1), 'zdata', zcirc(:,1), 'facecolor', [1, 0.8, 0.6], 'linewidth', 2);
pointA = line('xdata', xA(1), 'ydata', yA(1), 'zdata', zA(1), 'marker', 'o', 'color', 'r', 'markerfacecolor', 'r', 'linewidth', 3);

%  Animate the disk's motion by updating the figure with its current 
%  location and orientation:

pause

% animation = VideoWriter('rolling-disk.avi');
% animation.FrameRate = 1/dt;
% open(animation);

for k = 1:length(xP)
    set(path, 'xdata', xP(1:k), 'ydata', yP(1:k), 'zdata', zP(1:k));
    set(disk, 'xdata', xcirc(:,k), 'ydata', ycirc(:,k), 'zdata', zcirc(:,k));
    set(pointA, 'xdata', xA(k), 'ydata', yA(k), 'zdata', zA(k));
    drawnow   
    % writeVideo(animation, getframe(gcf));
end 

% close(animation);