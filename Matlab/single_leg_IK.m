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

%IK Simulate
Leg_1.inverse(75, 0, -138);

Leg_1.updateBody();
Leg_1.updatePlot(true);

ee_coords    = Leg_1.ee
joint_angles = rad2deg(Leg_1.theta)
