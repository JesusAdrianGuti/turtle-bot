function [mean_col, area] = find_robot(rgbImg);

%% function to find the mean column and the width of the bounding box
% of the image area where the robot appears in the image

% this is a specialized function, will only work for tracking 
% in the Empty Gazebo world simulator

% find non-background pixels 
Im_top = rgbImg(1:265,:,1)~=178;
Im_bot = (rgbImg(266:480,:,1)~=155).*(rgbImg(266:480,:,1)~=80);

% form the image that contains the non-background pixels
Im_bw = [Im_top;Im_bot];

figure(20)
imshow(Im_bw)

% find the nonzero elements
[rows,cols] = find(Im_bw);

% the function returns empty variables if the robot is not found
if isempty(cols)
	mean_col = [];
	area = [];
else
	mean_col = mean(cols);
	area = length(cols);
end





