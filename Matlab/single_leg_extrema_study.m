clear
cla

len          = [27.0, 48.0, 153.0];
offset_angle = deg2rad(0);
offset_coord = [0, 0, 0];
axis_limits  = [0                       , len(3)+len(2)+len(1) ...
                -len(3)-len(2)-len(1)   , len(3)+len(2)+len(1) ...
                -len(3)-len(2)          , len(3)+len(2)];            
Leg_1        = Leg(len, offset_coord, offset_angle);
Leg_1.createPlot(axis_limits);
  
%FK Simulate
x = [];
y = [];
z = [];
for theta1 = -pi/2 : pi/12 : pi/2
    for theta2 = -pi/3 : 0.25 : pi/2
        for theta3 =  0 : 0.25 : pi*(5/6)
            Leg_1.forward(theta1, theta2, theta3);
            Leg_1.updateBody()
            cla
            plot3(Leg_1.body(1, :), Leg_1.body(2, :), Leg_1.body(3, :),...
                    '-bo', ...
                    'LineWidth',2,...
                    'MarkerSize', 4,...
                    'MarkerFaceColor', 'blue');
            pause(0.0001)
            x = [x Leg_1.body(1, :)];
            y = [y Leg_1.body(2, :)];
            z = [z Leg_1.body(3, :)];
        end
    end
end

figure(2);
subplot(2,2,[3 4]);
scatter3(x, y, z, 'k.');
axis equal
grid on
xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');
zlabel('Z-axis (mm)');
title('Orthographic');
subplot(2,2,1);
scatter3(x, y, z, 'k.');
view(0,0);
axis equal
grid on
xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');
zlabel('Z-axis (mm)');
title('XZ plane');
subplot(2,2,2);
scatter3(x, y, z, 'k.');
view(0,90);
axis equal
grid on
xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');
zlabel('Z-axis (mm)');
title('XY plane');
