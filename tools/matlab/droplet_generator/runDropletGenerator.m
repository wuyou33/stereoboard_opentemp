clc;
global speed angular_speed delfly_width safety_width turn_radius ...
        extra_delay minimum_delay BT_delay view_angle...
        Y_max min_disp max_disp rows cols scale_factor...
        offset_x offset_y q03 q13 q23 q32 q33
         
speed = 550;                % [mm/s]
angular_speed = 1;          % [rad/s]
delfly_width = 140 ;        % [mm]
safety_width = 300 ;        % [mm]
turn_radius = 500  ;        % [mm]
extra_delay = 0; %0.20*speed ;  % [mm]
minimum_delay = 0; %0.25*speed; % [mm]
BT_delay = 0.0*speed;      % [mm]
view_angle = 55.7*pi/180;     % [deg] measured:56

Y_max = 400; % [mm] vertical size of droplet [-Y_max Y_max]
min_disp = 0;
max_disp = 16;

rows = 96;
cols = 128;

scale_factor = 6; 
offset_x = cols/2;
offset_y = rows/2;



maximum_disparities = region_Droplet_DelFly_Explorer;

%%

string_disp = num2str(maximum_disparities(1));
for i = 1:rows
    for j = 1:cols
        string_disp = [string_disp ','  num2str(maximum_disparities(i,j)) ];
    end
end

string_disp = string_disp(4:end)

