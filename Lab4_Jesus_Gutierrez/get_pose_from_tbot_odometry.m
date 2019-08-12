function  curr_pose = get_pose_from_tbot_odometry(tbot)

global robot_poses
persistent call_count

%this counts how many times we've received odometry
if isempty(call_count)
	call_count=1;
end

% get the odometry data
[odom,odomdata] = getOdometry(tbot);

pose = odomdata.Pose.Pose;
% x position of the robot in the world frame
x = odomdata.Pose.Pose.Position.X;
% y position of the robot in the world frame
y = odomdata.Pose.Pose.Position.Y;

% quaternion of the robot's orientation in the world frame
q= odomdata.Pose.Pose.Orientation;
% get the angle from the quaternion
phi = 2*atan2(q.Z,q.W);

% get the timestamp of the odoemtry reading
time = odomdata.Header.Stamp;
% get the time in seconds
t = time.Sec + time.Nsec/1e9;

 
robot_poses(:,call_count) = [x;y;phi;t];
call_count = call_count+1 ;

% return current pose also
curr_pose = [x;y;phi];