clear all
close all

% initialize
rosshutdown % to 'close' any previous sessions 
rosinit('192.168.0.103'); % initialize Matlab ROS node
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

%% LAB 4 Part 2 - Following the Cylinder

while 1
    
    rgbImg = getColorImage(tbot);
    
  % CHOOSE CYLINDER COLOR
  % cyl = 1 for GREEN
  % cyl = 2 for ORANGE
    cyl = 2 % ( 1 or 2)
  
    
    
 %% Green Cylinder Detection
    if (cyl == 1)
        
        % We need to change the Image into HSV coordinate system
        % image to hsv
        hsvImg = rgb2hsv(rgbImg);
    
        % Now turn this in to a Binary Image, testing only the HUE (color)value
        %h=hsvImg(:,:,1); %surf(h);shading interp **Use commands to find the Range Values
         
    
        % Binary Image
        % hsvImg(:,:,1) will give us the Hue Value.
        B_I = (hsvImg(:,:,1) > 0.2335).*(hsvImg(:,:,1) < 0.2875);
        figure(30)
        imshow(B_I) % Displays Binary image
        
        % If the tbot doesnt detect the green cylinder anywhere in its view
        % then bot should ratate in place
        if norm(B_I) == 0
            lin_vel = 0;
            rot_vel = 0.35;
            
        % Else track cylinder
        else 
        % Use Region Properties to find center & area of cylinder
        cen = regionprops(B_I,'centroid');
        x = cen.Centroid;
        % P-Controller for Rotational Velocity. centroid info 
        Kp = 0.00099;
        delta = x(1) - 320;
        % if we are close to middle slow down rot_vel
          if (delta <= 48) 
            Kp = Kp * (0.55);
          end
        rot_vel = -Kp * delta;
    
        %linear P-Controller. area info
        % Dont move forward unless cylinder is in view and centered
          if abs(delta) < 120
            area = regionprops(B_I,'Area');
            a = area.Area;
            Kl = 0.000012;
            dist = a - 23000; 
            lin_vel = -Kl * dist;
            pause (0.1)
          else
             lin_vel = 0.09;
          end
        end
        
    end 
    
    %% Orange Cylinder Detection 
    if (cyl == 2)
        % Orange Cylinder. 
        % Better detected using Saturation & Hue Values combined
    
        hsvImg = rgb2hsv(rgbImg);
        
        % Binary Image
        Sat = (hsvImg(:,:,2) > 0.68).*(hsvImg(:,:,1) < 0.90);
        Hue = ((hsvImg(:,:,1) < 0.075));
        both = Hue.*Sat ;
        imshow(both)    % Display Binary image
    
         if norm(both) == 0
            lin_vel = 0;
            rot_vel = 0.35;
        else
        % Use Region Properties to find center & area of cylinder
        cen = regionprops(both,'centroid');
        x = cen.Centroid;
        % P-Controller for Rotational Velocity. centroid info 
        Kp = 0.00099;
        delta = x(1) - 320;
          if (delta <= 48)  % if we are close to middle slow down rot_vel
            Kp = Kp * (0.55);
          end
        rot_vel = -Kp * delta;
    
        %linear P-Controller. area info
        % Dont move forward unless cylinder is in view and centered
          if abs(delta) < 120
            area = regionprops(both,'Area');
            a = area.Area;
            Kl = 0.000012;
            dist = a - 23000; 
            lin_vel = -Kl * dist;
            pause(0.1)
          else
            lin_vel = 0.09;
          end
        end
       
	 end
    
end
