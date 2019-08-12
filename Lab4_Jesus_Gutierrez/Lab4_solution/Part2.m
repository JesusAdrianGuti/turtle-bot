clear all
close all
 
% initialize
rosshutdown % to 'close' any previous sessions 
rosinit('192.168.0.102'); % initialize Matlab ROS node
tbot = turtlebot  % the data structure that allows access to the turtlebot and its sensors
    
% these are the variables that are used to define the robot velocity
lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 

% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
% This will create a new thread that runs the command setVelocity(tbot,lin_vel,rot_vel) every 100msec
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.1,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)
tbot.ColorImage.TopicName = '/usb_cam/image_raw/compressed';  

kp = 0.003;
imgcount = 0;
while 1
	tic
	% get an image from the camera
	rgbImg = getColorImage(tbot);
	% display the image
	figure(10)
	imshow(rgbImg)
	
	% turn the image to hsv
	hsv = rgb2hsv(rgbImg);
        
    % get binary image from hue, saturation and value. Note that the threshold
    % using below works well on my test case. However, different test
    % environment generally has different threshold values, where you can
    % get hint from the file hsv_analysis.m
    green_binary_hue = (hsv(:,:,1) > 0.27).* (hsv(:,:,1) < 0.39); % hue
    green_binary_satu = (hsv(:,:,2) > 0.37).* (hsv(:,:,2) < 0.80); % saturation
    green_binary_value = (hsv(:,:,3) > 0.3).* (hsv(:,:,3) < 0.60); % value

    % final binary image after thresholding
    green_binary_hsv = green_binary_hue.* green_binary_satu.* green_binary_value;

    if length(find(green_binary_hsv)) < 1000 % if object not big enough, see as outlier
        lin_vel = 0;
        rot_vel = 0.5;
    else
        % detect the object
        figure(10)
        imshow(green_binary_hsv)
        middle_col = size(green_binary_hsv, 2)/2;
        % object centroid
        stat = regionprops(green_binary_hsv, 'Centroid');
        centroid = stat.Centroid;
        centroid(1)
        lin_vel = 0;
        rot_vel = -kp* (centroid(1) - middle_col);
        imgcount = imgcount+ 1;

    end
	toc
end
	
  