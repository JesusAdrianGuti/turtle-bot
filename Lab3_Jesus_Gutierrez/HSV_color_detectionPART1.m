clear all
close all
 
% initialize
rosshutdown % to 'close' any previous sessions 
rosinit('172.16.144.132'); % initialize Matlab ROS node
tbot = turtlebot  % the data structure that allows access to the turtlebot and its sensors
    
% these are the variables that are used to define the robot velocity
lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 

% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
% This will create a new thread that runs the command setVelocity(tbot,lin_vel,rot_vel) every 100msec
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.1,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)
  

% handle to the simulator
gazebo = ExampleHelperGazeboCommunicator();

% create a few colored balls in the environment
ball = ExampleHelperGazeboModel('Ball')
spherelink = addLink(ball,'sphere',1,'color',[0.1 1 0 1])
spawnModel(gazebo,ball,[6.5,1,1]);

ball2 = ExampleHelperGazeboModel('Ball')
spherelink = addLink(ball2,'sphere',1,'color',[1 1 0 1])
spawnModel(gazebo,ball2,[5.5,2,1]);

ball3 = ExampleHelperGazeboModel('Ball')
spherelink = addLink(ball3,'sphere',1,'color',[0 1 1 1])
spawnModel(gazebo,ball3,[7.5,-1,1]);

ball4 = ExampleHelperGazeboModel('Ball')
spherelink = addLink(ball4,'sphere',1,'color',[1 0 0 1])
spawnModel(gazebo,ball4,[6.5,-2,1]);
  



while 1
	tic
	% get an image from the camera
	rgbImg = getColorImage(tbot);
	% display the image
	figure(10)
	imshow(rgbImg)
	
	% turn the image to hsv
	hsvImg = rgb2hsv(rgbImg);
	
	%% YOUR CODE HERE
    toc
    % LAB 3 - PART 1 - Need to Find the Green Ball & Make it the center
    
    % First make a Binary Image.To do this we will test the Hue(Color) value
    % Range 0.17 < H < 0.34 is the green range.Anything outside we disregard
	B = (hsvImg(:,:,1) > 0.17).*(hsvImg(:,:,1) < 0.34);
    
    %Plotting Binary Img
    figure(20)
    imshow(B)
    
    % Get Properties from this binary Image using 'regionprops()' function
	c =  regionprops(B, 'centroid');
    xval = c.Centroid;
 
    % Use P-Controller to get  the bot to rotate
    % The Center of the Image is at (320,240)
    Kp= 0.00026; % Test For best Value
   
    % P-Controller
    delta = xval(1) - 320; 
    rot_vel =  -Kp * delta;
     
end
	
 