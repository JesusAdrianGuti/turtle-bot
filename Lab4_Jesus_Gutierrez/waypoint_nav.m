clear all
close all

% initialize
rosshutdown % to 'close' any previous sessions 
rosinit('192.168.139.130'); % initialize Matlab ROS node
tbot = turtlebot  % the data structure that allows access to the turtlebot and its sensors

% this command sets the current robot pose estimate to zero (it does not
% move the robot). This is equivalent to setting the ``world coordinate
% frame'' to coincide with the current robot frame.
resetOdometry(tbot)

% the variable robot_poses stores the history of robot pose received from odometry. It  
% is declared as global for computational efficiency
global robot_poses
% we initialize robot_poses as an empty matrix with 10000 columns. 
% This can store up to 1000 seconds of odometry, received at 10Hz. 
robot_poses = zeros(4,10000);

% create a Matlab "timer" for receiving the odometry. This will effectively create a new thread
% that executes 'curr_pose = get_pose_from_tbot_odometry(tbot);' every 100 msec. 
% this will return the current robot pose in the variable curr_pose, and
% will also store the current pose, together with its timestamp, in robot_poses 
odometry_timer = timer('TimerFcn','curr_pose = get_pose_from_tbot_odometry(tbot);','Period',0.1,'ExecutionMode','fixedRate');
% start the timer. This will keep running until we stop it.
start(odometry_timer)
   
% these are the variables that are used to define the robot velocity
lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 

% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
% This will create a new thread that runs the command setVelocity(tbot,lin_vel,rot_vel) every 50msec
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.05,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)
 

%% The waypoints for moving on a square:
% we make the robot move on a square
% the size of the square
square_size = 1;

% generate the waypoints
dp = .5; % spacing between waypoints
pp = [0:0.5:square_size];

waypoints = [pp;zeros(size(pp))];
waypoints = [waypoints [square_size*ones(size(pp));pp]];
waypoints = [waypoints [pp(end:-1:1);square_size*ones(size(pp))]];
waypoints = [waypoints [zeros(size(pp));pp(end:-1:1)]];

  
% create a Matlab timer for plotting the trajectory
plotting_timer = timer('TimerFcn','plot_trajectory(robot_poses, waypoints)','Period',5,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(plotting_timer)
 

% the variable "waypoints" is a 2xN matrix, which contains the x-y
% coordinates of the N waypoints that the robot needs to go to. 
% these coordinates are in the "world" coordinate frame.

% the number of waypoints 
N_waypoints = size(waypoints,2);

% the index to the current goal waypoint
curr_waypoint = 1;
% distance threshold to decide when we've reached a waypoint
d_thresh = 0.03;
% controller gain
k_p = 2;
 
while 1
	
	% keep doing this while the distance to the current goal waypoint is
	% larger than d_thresh
	while norm(curr_pose(1:2)-waypoints(:,curr_waypoint))>d_thresh
		
		% find waypoint's position in the local frame:
		% this is the rotation matrix expressing the robot's orientation in
		% the world coordinate frame. 
		R = [cos(curr_pose(3)) -sin(curr_pose(3)) 0 ;
		     sin(curr_pose(3)) cos(curr_pose(3))  0
			 0                      0             1];
			     
	    R_p  = R'*([waypoints(:,curr_waypoint)-curr_pose(1:2) ; 0]);
		
		% find the angle theta
		theta = atan2(R_p(2),R_p(1)); 
		
		% define the velocities
		% make sure the robot does not turn too fast... we don't want the
		% netbook to fly off
		rot_vel = sign(theta)* min(abs(k_p* theta), 1);

		% the linear velocity is limited to 0.25 m/sec. When the rotational
		% velocity is large (i.e., theta is large), we reduce the linear
		% velocity, to make sure there is enough time to turn towards the next
		% waypoint.
		lin_vel = min(0.25, max(0,0.25-0.5*abs(rot_vel)));

		pause(0.05)
	
	end
	
	% increase the waypoint index
	curr_waypoint = curr_waypoint+1;
	if curr_waypoint == N_waypoints+1;
		curr_waypoint=1;
	end
	
end

 
  