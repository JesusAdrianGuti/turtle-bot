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
  
% create a second robot in the Gazebo simulator
% handle to the simulator
gazebo = ExampleHelperGazeboCommunicator();
% the robot model
botmodel = ExampleHelperGazeboModel('turtlebot','gazeboDB') 
% spawn the model in gazebo
bot = spawnModel(gazebo,botmodel,[3,0,0]) 
% set the initial orientation of the robot
setState(bot,'orientation',[0 0 pi/3]) 
% get the handle to the robot's joints (we'll need this to make the robot
% move)
[botlinks, botjoints] = getComponents(bot)

% pause so that everything has time to be initialized
pause(5)

% the number of images we have processed
image_count=0;

% create a Matlab timer for moving the second robot
robot_move_timer = timer('TimerFcn','move_robot(image_count,bot,botjoints)','Period',10,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(robot_move_timer)
%% LAB 3 PART 2 - FOLLOWING 2ND BOT USING CAMERA INPUTS

while 1
	tic
	% get an image from the camera
	rgbImg = getColorImage(tbot);
	figure(10)
	imshow(rgbImg)
	
	% find the robot in the image. This function returns the column where
	% the middle of the robot will appear in the image, and the number of
	% pixels that belong to the robot in the image
	[mean_col, area] = find_robot(rgbImg);
    
	%YOUR CODE HERE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                          
    Kr = 0.00165; % Constant for rotational velocity
                  % We use a high rot_vel vales in order to not lose the
                  % target out of view.Rotating faster when its out of center
    Kl = 0.00018; % Constant for linear velocity
    
    delta = mean_col - 320; % rotational delta
    dist = area - 4000;     % linear delta
    
    % if the target is close to middle of image, slow down rotation velocity 
    if (delta <= 48) 
            Kr = Kr * (0.55);
    end
    
    % if we dont see the target, bot should rotate in place until its found
    if isempty(mean_col) 
        rot_vel = -0.65; % radians per second
        lin_vel = 0.0; 
    else 
        % If we do see the target, 
        % want to try to keep the target at the center of image at all times
        % center is at 320 pixels
        % delta = mean_col - 320
    rot_vel =  -Kr * delta; % P-Controller for Rotational Velocity
    
        % Want to follow the target & stay a relative constant distane from it
        % The target should look the same size (or area in our pictures)
        % dist = area - 4000;
    lin_vel = -Kl * dist;  % P-Controller for Linear Velocity
     
	% show the elapsed time for a loop
	toc
	% the number of images we processed
	image_count=image_count+1;
    end
end
	
