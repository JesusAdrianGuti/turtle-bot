function plot_trajectory(robot_poses, waypoints)

max_ind = max(find(robot_poses(4,:)));
figure(121)
if nargin==2
	
	plot(waypoints(1,:),waypoints(2,:),'r*-')
	hold on
end
plot(robot_poses(1,1:max_ind),robot_poses(2,1:max_ind),'b.')
axis equal
xlabel('x (m)')
ylabel('y (m)')
title('Robot trajectory')




figure(122)
subplot(3,1,1)
plot(robot_poses(4,1:max_ind)-robot_poses(4,1), robot_poses(1,1:max_ind))
ylabel('x (m)')
title('Robot pose over time ')
subplot(3,1,2)
plot(robot_poses(4,1:max_ind)-robot_poses(4,1), robot_poses(2,1:max_ind))
ylabel('y (m)')
subplot(3,1,3)
plot(robot_poses(4,1:max_ind)-robot_poses(4,1),180/pi*robot_poses(3,1:max_ind))
xlabel('Time (sec)')
ylabel('Angle (deg)')
 
