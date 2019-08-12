clear all
close all

% initialize
rosshutdown % to 'close' any previous sessions 
rosinit('192.168.0.100'); % initialize Matlab ROS node
tbot = turtlebot('192.168.0.100')  % the data structure that allows access to the turtlebot and its sensors
tbot.ColorImage.TopicName = '/usb_cam/image_raw/compressed'; % set netbook webcam as the camera for tbot
% these are the variables that are used to define the robot velocity
lin_vel = 0.00;  % meters per second
rot_vel = 0.00;  % rad/second 
% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.05,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)

Kr = 0.00195;
Kl = 0.00000041;
condition = 1; % Once final position is met the iterations, and velocities will stop
a = 0;         % Condition = 0 once docked, otherwise will be 1  
while condition
    % obtain image from bot's camera
	rgbImg = getColorImage(tbot);
    % image to hsv
    hsv = rgb2hsv(rgbImg);
         
    % Create binary Matrices for each value of HSV
    hue = (hsv(:,:,1) > 0.16).* (hsv(:,:,1) < 0.44); % hue filter range
    sat = (hsv(:,:,2) > 0.23).* (hsv(:,:,2) < 0.69);  % saturation filter range
    val = (hsv(:,:,3) > 0.17).* (hsv(:,:,3) < 0.88); % value filter range
    
    % combine all matrices to get best binary image with least amount of noise
    bin = hue.* sat .* val; 
    best = bwareafilt(imbinarize(bin),1); %optimal binary image using all the filters
    
    figure(30)
    imshow(best) % Displays Binary image
     
    % If the tbot doesnt detect the green cylinder anywhere in its view
    % then bot should ratate in place
    if (length(find(best))) < 250
        lin_vel = 0.0;
        rot_vel = 0.38;
            
    % Else track cylinder
    else 
        % Use Region Properties to find center & area of cylinder
        center = regionprops(best,'centroid');
        x = center.Centroid;
        %¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬
        % Rotational P-Controller
        delta = x(1) - 320;
        % Once close to middle of image, slow down rot_vel to be more accurate
        if (abs(delta) <= 48)
            Kr = 0.0011;
        end 
        rot_vel = -Kr * delta;
        %¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬
        % Linear P-Controller
        
        area = regionprops(best,'Area');
        a = area.Area;
        % Dont move forward unless cylinder is in view and centered
        if abs(delta) < 240
              dist = a - 307200; 
              if ( abs(dist) < 30500 )
                 Kl = 0.00000001;
              end
              lin_vel = -Kl * dist;
        else
            lin_vel = 0.04;
        end
    end            
    % once final position is achieved
    if( a > 277000 ) 
        rot_vel = 0;
        lin_vel = 0;
        condition = 0;
    end

end
