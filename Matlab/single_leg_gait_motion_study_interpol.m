%% Object Creation
clear
cla

len          = [27.0, 48.0, 153.0];
offset_angle = deg2rad(0);
offset_coord = [0, 0, 0];
axis_limits  = [0                       , len(3)+len(2)+len(1) ...
                -len(3)-len(2)-len(1)   , len(3)+len(2)+len(1) ...
                -len(3)-len(2)          , len(3)+len(2)];            
Leg_1        = Leg(len, offset_coord, offset_angle);
Leg_2        = Leg(len, offset_coord, offset_angle);
Leg_1.createPlot(axis_limits);

%% Gait Cycle Coordinates
%bottom center of the gait cycle as an offset from ee neutral position
gait_x0 = 25;
gait_y0 =  0;
gait_z0 = 25;
naught  = [ len(1) + len(2) + gait_x0, 0, -len(3) + gait_z0];

%lengths of the rectangle which defines the gait and direction of travel
delta_y   = 40; 
delta_z   = 20;
direction = 0;
del     = transpose(rotz(direction)*[   0;   delta_y;  delta_z]);

%coordinates of the gait in the ground frame
%        3---------dy--------2
%        |                   |
%        dz                  dz
%        |                   |
%        4---------0---------1              
loc(1, :) = [naught(1) - del(1), naught(2) - del(2), naught(3)];
loc(2, :) = [naught(1) - del(1), naught(2) - del(2), naught(3) + del(3)];
loc(3, :) = [naught(1) + del(1), naught(2) + del(2), naught(3) + del(3)];
loc(4, :) = [naught(1) + del(1), naught(2) + del(2), naught(3)];
Leg_1.inverse(loc(1, 1), loc(1, 2), loc(1, 3))

%% Driver and animation
t_stance = 5;
t_stride = 2;
t_period = t_stance + t_stride;

t_epoch = tic;
while true
    t = mod(toc(t_epoch), t_period);
    if t >= 0 & t < t_stride
        if     t < t_stride*(1/3)
            ti = 0;
            current = sine_interpol(loc(1,:), loc(2,:), t, t_stride/3, ti);
        elseif t < t_stride*(2/3)
            ti = t_stride*(1/3);
            current = sine_interpol(loc(2,:), loc(3,:), t, t_stride/3, ti);
        elseif t < t_stride*(3/3)
            ti = t_stride*(2/3);
            current = sine_interpol(loc(3,:), loc(4,:), t, t_stride/3, ti);
        end
    elseif t >= t_stride & t < t_period
        ti = t_stride;
        current = linear_interpol(loc(4,:), loc(1,:), t, t_stance, ti);
    end
    Leg_1.inverse(current(1), current(2), current(3));
    Leg_1.updateBody()
    Leg_1.updatePlot(true);
    pause(0.01);  
end

%% Sinusoidal Interpolation
function current3 = sine_interpol(initial3, final3, t_curr, t_span, t_initial)
    for i = 1:3
        initial = initial3(i);
        final  = final3(i);
        
        C = final - initial;
        t = (t_curr - t_initial)/t_span;
        current3(i) = C*(t - sin(2*pi*t)/(2*pi)) + initial;
    end
end

function current3 = linear_interpol(initial3, final3, t_curr, t_span, t_initial)
    for i = 1:3
        initial = initial3(i);
        final  = final3(i);
        
        t_final     = t_span  + t_initial;
        numerator   = final   - initial;
        denominator = t_final - t_initial; 
        current3(i) = (numerator/denominator)*(t_curr - t_initial) + initial;
    end
end
