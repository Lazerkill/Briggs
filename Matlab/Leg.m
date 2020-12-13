classdef Leg < handle
    properties
        len             %joint lengths
        offset_coord    %ground frame offset coordinates     
        offset_angle    %ground frame offset angle
        ee              %end effector coordinates
        theta           %joint angles
        body            %defines every point in the leg
        f
    end
    
    methods
        function obj = Leg(len, offset_coord, offset_angle)
            obj.len(1:3)      = len(1:3);
            obj.offset_coord  = offset_coord(1:3);
            obj.offset_angle  = offset_angle;   
            obj.ee            = [len(1) + len(2), 0, -len(3)];
            obj.theta         = [0, 0, pi/2];
        end       
        
        %Updates joint angles given end effector coordinates
        function inverse(obj, x, y, z)
            obj.ee = [x, y, z];
            
            obj.theta(1) = atan2(y, x); 

            numerator   = (sqrt((x)^2 + (y)^2) - obj.len(1))^2 + z^2 - obj.len(3)^2 - obj.len(2)^2;
            denominator = 2.0 * obj.len(2) * obj.len(3);
            obj.theta(3)  = acos(numerator/denominator);
            
            Q           = sqrt((x)^2 + (y)^2) - obj.len(1);
            numerator   = Q*(obj.len(3)*sin(obj.theta(3))) + z*(obj.len(3)*cos(obj.theta(3)) + obj.len(2));
            denominator = Q*(-obj.len(3)*cos(obj.theta(3)) - obj.len(2)) + z*(obj.len(3)*sin(obj.theta(3)));
            obj.theta(2)  = atan(numerator/denominator);
            
        end
        
        %Updates end effector coordinates given joint angles
        function forward(obj, theta1, theta2, theta3)
            obj.theta = [theta1, theta2, theta3];
            
            T_offset =          makehgtform('translate', obj.offset_coord, 'zrotate', obj.offset_angle);
            T_01     = T_offset*makehgtform('zrotate', obj.theta(1), 'translate', [obj.len(1) 0 0], 'xrotate', -pi/2);
            T_12     =          makehgtform('zrotate', obj.theta(2), 'translate', [obj.len(2) 0 0]);
            T_23     =          makehgtform('zrotate', obj.theta(3), 'translate', [obj.len(3) 0 0]);
            
            T_03 = T_01*T_12*T_23;
            
            obj.ee(1) = T_03(1, 4);
            obj.ee(2) = T_03(2, 4);
            obj.ee(3) = T_03(3, 4);
           
        end
        
        %Updates the coordinates for every point in the leg
        function updateBody(obj)
            T_offset = makehgtform('translate', obj.offset_coord, 'zrotate', obj.offset_angle);
            T_01     = T_offset*makehgtform('zrotate', obj.theta(1), 'translate', [obj.len(1) 0 0], 'xrotate', -pi/2);
            T_12     = makehgtform('zrotate', obj.theta(2), 'translate', [obj.len(2) 0 0]);
            T_23     = makehgtform('zrotate', obj.theta(3), 'translate', [obj.len(3) 0 0]);
            
            T_02 = T_01*T_12;
            T_03 = T_01*T_12*T_23;
            
            x = [obj.offset_coord(1), T_01(1, 4), T_02(1, 4), T_03(1, 4)];
            y = [obj.offset_coord(2), T_01(2, 4), T_02(2, 4), T_03(2, 4)];
            z = [obj.offset_coord(3), T_01(3, 4), T_02(3, 4), T_03(3, 4)];
            obj.body = [x; y; z];
        end
        
        %Creates graphics for a single leg simulations
        function createPlot(obj, axis_limits)
            obj.f = figure(1);
            
            plot3(0, 0, 0);    
            hold on
            axis equal
            axis(axis_limits);
            grid on
            xlabel('X-axis (mm)');
            ylabel('Y-axis (mm)');
            zlabel('Z-axis (mm)');
            
            set(obj.f,'Name', strcat('Single Leg Simulation: ',int2str(i)), 'Visible', 'on', 'NumberTitle', 'off');
        end
        
        %Updates plot. Garbage collection on for live animation.
        function updatePlot(obj, garbage_collection)
            if garbage_collection == true
                cla
            end
            plot3(obj.body(1,:), obj.body(2,:), obj.body(3,:),...
                                '-bo', ...
                                'LineWidth', 1,...
                                'MarkerSize', 4,...
                                'MarkerFaceColor', 'red',...
                                'MarkerEdgeColor', 'red');
        end
    end
end