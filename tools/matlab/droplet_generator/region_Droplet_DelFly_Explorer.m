function [disparity_maximum] = region8

% Region to circle safely and make an error

global speed angular_speed delfly_width safety_width turn_radius ...
        extra_delay minimum_delay BT_delay view_angle...
        Y_max min_disp max_disp rows cols scale_factor...
        offset_x offset_y q03 q13 q23 q32 q33

%% point list

% DelFly at origin
point1 = [0;0];


width = delfly_width+safety_width;
total_radius = width+turn_radius+minimum_delay;


%  Circle
circle_1_center_x = 0;
circle_1_center_y = total_radius/sin(view_angle/2);

circle_1_start = -view_angle/2;
circle_1_end = pi+view_angle/2;

t = circle_1_start:0.01:circle_1_end;
x = total_radius*cos(t) + circle_1_center_x;
y = total_radius*sin(t) + circle_1_center_y;

% y(t>0) = y(t>0)+extra_delay;
pointsC1 = [x;y];


% inner parcours
parcour_radius = turn_radius;
start_angle = asin(parcour_radius/circle_1_center_y);

cam_angle = start_angle*180/pi

circle_1_start = pi+start_angle;
circle_1_end = pi+start_angle+2*pi;

t = circle_1_start:0.01:circle_1_end;
x = parcour_radius*cos(t) + circle_1_center_x;
y = parcour_radius*sin(t) + circle_1_center_y;
pointsT1 = [x;y];


passive_track_length = sqrt(x(1)^2+y(1)^2)


% outer parcours

parcour_radius = turn_radius+minimum_delay;
start_angle = asin(parcour_radius/circle_1_center_y);

circle_1_start = pi+start_angle;
circle_1_end = pi+start_angle+2*pi;

t = circle_1_start:0.01:circle_1_end;
x = parcour_radius*cos(t) + circle_1_center_x;
y = parcour_radius*sin(t) + circle_1_center_y;
pointsT2 = [x;y];

maxdispsLeft1 = ones(rows,cols)*max_disp*16;
maxdispsRight1 = maxdispsLeft1;

focal_length = 118;
base_line = 60;
cx_diff = -1.69731442088569;
dx = 1.43291742013321;

% x = [118.002055142877 -1.69731442088569 1.43291742013321 -1.67469840132892 4201.99972185017];
x = [118.006474777565 -0.695622042123951 0.919464478567257 -1.34553041476202 4201.99439180743];
f = x(1);
cx_diff = x(2);
ground_pos_correction = x(5);
dx = x(3);
dy = x(4);

for row = 1:rows
    for col = 1:cols
        for d = max_disp*scale_factor:-1:min_disp*scale_factor
            
            X = col - offset_x;
            Y = row - offset_y;
            Z = focal_length;
            W = (d/scale_factor)/base_line;
%             W = (d/scale_factor)/base_line -cx_diff/base_line + X*(dx/cols)/base_line;
            W = (d/scale_factor)/base_line - (X*(dx/cols)/base_line);
%             W = (d/scale_factor)/base_line - cx_diff/base_line ;
%             W = -(X*(dx/cols)/base_line);
            
            
            X = X / W;
            Z = Z / W;
            
            dx = 0;
            cx_diff = 0; % corrected for in algorithm
            [world_coordinates, indices] = DispconvertPoints_Correction(f, cx_diff, base_line, [col row d/scale_factor], cols, rows, 0, 0);

            X = world_coordinates(1);
            Y = world_coordinates(2);
            Z = world_coordinates(3);
                       
            if ( abs(Y)<Y_max )
                if ( X<0 && X>-total_radius )
                    max_Z = circle_1_center_y + sqrt(total_radius^2-(X)^2);                
                elseif ( X>=0 && X<total_radius )
                   max_Z = circle_1_center_y + sqrt(total_radius^2-(X)^2);
                else
                    max_Z = 0;
                end
                if ( Z<max_Z && Z > 0)
                    maxdispsLeft1(row,col) = d;                        
                end   
            end
             
        end
    end
end

%% calculate time-to-turn and frames-to-turn

TTT = round(( sqrt(  pointsT1(1,1)^2 +  pointsT1(2,1)^2 ) - minimum_delay + BT_delay )/speed*1000)
FTT = TTT/40

% TTT1 = (circle_5_center_y - minimum_delay + BT_delay )/speed
% TTT2 = ( sqrt((pointsT1(1,end)-pointsT5(1,1))^2 + (pointsT1(2,end)-pointsT5(2,1))^2 ) - minimum_delay + BT_delay )/speed;
% TTT2 =  circle_5_center_y/speed + (view_angle/2)/angular_speed + TTT2


% TTT3 = ( sqrt((pointsT3(1,end)-pointsT41(1,1))^2 + (pointsT3(2,end)-pointsT41(2,1))^2 ) - minimum_delay + BT_delay )/speed


%% plot and write

points1 = [point1 pointsC1 point1];
points2 = [pointsT1 point1];
points3 = pointsT2;

figure(1)
plot(points1(1,:)/10,points1(2,:)/10,'LineWidth',2)
hold on
plot(points2(1,:)/10,points2(2,:)/10,'-.k')
plot(points2(1,2)/10,points2(2,2)/10,'dr')

plot(points3(1,:)/10,points3(2,:)/10,'-.k')
% plot(circle_1_center_x/10,circle_1_center_y/10,'.g')
xlabel('x [cm]')
ylabel('z [cm]')
legend('region bounday','avoidance trajectory','turn point','Location','SouthEast')
% plot(circle_2_center_x,circle_2_center_y,'.g')
% plot(circle_3_center_x,circle_3_center_y,'.g')
% plot(circle_4_center_x,circle_4_center_y,'.g')
% plot(circle_5_center_x,circle_5_center_y,'.g')
ylim([0 350])
hold off
axis equal


Rtotal = turn_radius+delfly_width+safety_width
CPdist = Rtotal/sin(view_angle/2)
Awidth = 2*Rtotal
Alength = CPdist+Rtotal

figure(2)
[X,Y] = meshgrid(1:cols, 1:rows);    
surf(X,Y,maxdispsLeft1)

zlim([0 40])

% disparity_maximum = maxdispsLeft1(1,:);
disparity_maximum = maxdispsLeft1;

end
