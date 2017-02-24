function [ points3D,indices  ] = DispconvertPoints_Correction( f, cx_diff, baseline, data, width, height, dx, dy)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

indices = 1:size(data,1);

nr_of_points_to_exclude = 0;
%%
number_of_resulting_points = size(data,1)-nr_of_points_to_exclude;
if number_of_resulting_points < 0
    number_of_resulting_points = 0;
end
while size(data,1) > number_of_resulting_points
    max_data = (data(:,3) == max(data(:,3)));
    data = data(~max_data,:);
    indices = indices(~max_data);
end

nr_of_points_to_exclude = 0;
%%
number_of_resulting_points = number_of_resulting_points-nr_of_points_to_exclude;
if number_of_resulting_points < 0
    number_of_resulting_points = 0;
end
while size(data,1) > number_of_resulting_points
    min_data = (data(:,3) == min(data(:,3)));
    data = data(~min_data,:);
    indices = indices(~min_data);
end


%%

Q = [1 0 0 0 ; 0 1 0 0 ; 0 0 0 f; dx/width/baseline dy/height/baseline (-1/baseline) (cx_diff/baseline)];


points3D = data;
for r=1:size(data)
        X= [(data(r,1)-width/2)/1;(data(r,2)-height/2)/1;-data(r,3);1];
        
        conv= Q*X;
        
        points3D(r,:) = conv(1:3)/conv(4);
        
      
    
end
 

end

