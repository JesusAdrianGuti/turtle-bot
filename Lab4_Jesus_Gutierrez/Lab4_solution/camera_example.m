clear all
close all

% initialize
rosshutdown % to 'close' any previous sessions 
rosinit('192.168.0.102'); % initialize Matlab ROS node
tbot = turtlebot  % the data structure that allows access to the turtlebot and its sensors
tbot.ColorImage.TopicName = '/usb_cam/image_raw/compressed'; % set netbook webcam as the camera for tbot

% these are the variables that are used to define the robot velocity
lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 

% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
% This will create a new thread that runs the command setVelocity(tbot,lin_vel,rot_vel) every 50msec
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.05,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)

% the following loop reads and stores 20 images from the camera
for i = 1:20

	rgbImg = getColorImage(tbot);
 	imwrite(rgbImg,['image',num2str(i),'.bmp']);
	
end

% the following loop reads the stored images, and displays them
for i = 1:20

	rgbImg = imread(['image',num2str(i),'.bmp']);
 	figure(10)
	subplot(4,5,i)
	imshow(rgbImg)
	
end





 