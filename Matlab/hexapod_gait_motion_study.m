clear
cla

%% Geometry constants
len = [27.0, 48.0, 153.0];
frame_radius  = 104.24;
height_offset = 46.6;

%% Gait cycle coordinates
%bottom center of the gait cycle as an offset from ee neutral position
gait_x0 = 35;
gait_z0 = 15;
naught  = [ len(1) + len(2) + gait_x0, 0, -len(3) + gait_z0];

%lengths of the rectangle which defines the gait and direction of travel
delta_y   = 55; 
delta_z   = 35;
overall_direction = 60;
for i = 1:6
    ith_direction = (270 - (i*60)) + overall_direction;
    del           = transpose(rotz(ith_direction)*[   0;   delta_y;  delta_z]);
    loc(1, :, i) = [naught(1) - del(1), naught(2) - del(2), naught(3)];
    loc(2, :, i) = [naught(1) - del(1), naught(2) - del(2), naught(3) + del(3)];
    loc(3, :, i) = [naught(1) + del(1), naught(2) + del(2), naught(3) + del(3)];
    loc(4, :, i) = [naught(1) + del(1), naught(2) + del(2), naught(3)];
end
%coordinates of the gait in the ground frame
%        3---------dy--------2
%        |                   |
%        dz                  dz
%        |                   |
%        4---------0---------1              

%% Leg creation
for i = 1 : 6
   da   = i*pi/3;
   mount_offset = [frame_radius*cos(da), frame_radius*sin(da), height_offset];
   Legs(i) = Leg(len, mount_offset, da);
   Legs(i).inverse(naught(1), naught(2), naught(3));
end

%% Graphics creation
domain = 350;
axis_limits  = [-domain                          , domain ...
                -domain                          , domain ...
                -len(3) + height_offset + gait_z0, len(3)+len(2)+height_offset];       
f = createPlot(axis_limits);
updatePlot(Legs, 1:6);

%% Gait loop
t_stance = 3.0;
t_stride = 1.0;
t_period = t_stance + t_stride;

t_epoch = tic;
while true
    %toc(t_epoch)
    for i = 1:6
        t = mod(toc(t_epoch) + (i/6)*t_period, t_period);
        if t >= 0 & t < t_stride
            if     t < t_stride*(1/3)
                ti = 0;
                current = sine_interpol(loc(1,:,i), loc(2,:,i), t, t_stride/3, ti);
            elseif t < t_stride*(2/3)
                ti = t_stride*(1/3);
                current = sine_interpol(loc(2,:,i), loc(3,:,i), t, t_stride/3, ti);
            elseif t < t_stride*(3/3)
                ti = t_stride*(2/3);
                current = sine_interpol(loc(3,:,i), loc(4,:,i), t, t_stride/3, ti);
            end
        elseif t >= t_stride & t < t_period
            ti = t_stride;
            current = linear_interpol(loc(4,:,i), loc(1,:,i), t, t_stance, ti);
        end
        Legs(i).inverse(current(1), current(2), current(3));
        Legs(i).updateBody()
        cla
        updatePlot(Legs, 1:6);
        pause(0.01);  
    end
end

%% Helper functions
function updatePlot(Legs, staticLegs)
   for i = staticLegs
       Legs(i).updateBody()
       x = Legs(i).body(1, :);
       y = Legs(i).body(2, :);
       z = Legs(i).body(3, :);
       plot3(x, y, z,...
                                '-bo', ...
                                'LineWidth', 1,...
                                'MarkerSize', 4,...
                                'MarkerFaceColor', 'red',...
                                'MarkerEdgeColor', 'red');
   end
   
   for i = 1:6
       frame(i, 1:2) = Legs(i).offset_coord(1:2);
       frame(i, 3)   = 0;
       txt = sprintf('%x', i);
       text(frame(i, 1), frame(i, 2), frame(i,3)+25, txt, ...
                                                'FontSize', 10, ...
                                                'Color', 'magenta'); 
   end
   x = [frame(:, 1); frame(1, 1)];
   y = [frame(:, 2); frame(1, 2)];
   z = [frame(:, 3); frame(1, 3)];
   plot3(x, y, z, '-k', ...
                  'LineWidth', 1);
   
end

function f = createPlot(axis_limits)
            f = figure(1);
            
            plot3(0, 0, 0);    
            hold on
            axis equal
            axis(axis_limits);
            grid on
            xlabel('X-axis (mm)');
            ylabel('Y-axis (mm)');
            zlabel('Z-axis (mm)');
            
            set(f,'Name', 'Hexapod Simulation', 'Visible', 'on', 'NumberTitle', 'off');
end

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
