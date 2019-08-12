clear all
close all
rosshutdown % to 'close' any previous sessions 

% initialize
rosinit('192.168.1.66'); % initialize Matlab ROS node
tbot = turtlebot('192.168.1.66') % the data structure that allows access to the turtlebot and its sensors
tbot.ColorImage.TopicName = '/usb_cam/image_raw/compressed'; % set netbook webcam as the camera for tbot

global robot_poses;
resetOdometry(tbot)
robot_poses = zeros(4,5000);

% create a Matlab "timer" for receiving the odometry. 
odometry_timer = timer('TimerFcn','curr_pose = get_pose_from_tbot_odometry(tbot);','Period',0.1,'ExecutionMode','fixedRate');
start(odometry_timer) %This will keep running until we stop it.

lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 
% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.05,'ExecutionMode','fixedSpacing');
start(velocity_command_timer) % start the timer. This will keep running until we stop it.

Kr = 0.00207; 
shoot = 0;

    % Find the Centroid of Goal
objective1 = 1; 
disp('Step 1: Find and center the goal') % Display steps 
while objective1
    
    % obtain image from bot's camera
	rgbImg = getColorImage(tbot);
    % image to hsv format
    hsv = rgb2hsv(rgbImg);
    
    % Create binary Matrices for each value of HSV
    greenhue = (hsv(:,:,1) > 0.12).* (hsv(:,:,1) < 0.499); % hue filter range
    greensat = (hsv(:,:,2) > 0.20).* (hsv(:,:,2) < 0.699);  % saturation filter range
    greenval = (hsv(:,:,3) > 0.13).* (hsv(:,:,3) < 0.899); % value filter range
    
    % combine all matrices to get best binary image with least amount of noise
    greenbin = greenhue.* greensat .* greenval; 
    greenbest = bwareafilt(imbinarize(greenbin),2); %optimal binary image using all the filters
    % Displays Binary image
    figure(30)
    imshow(greenbest)
   
    % If the tbot doesnt detect the green cylinders anywhere in its view
    % then bot should ratate in place
    if (length(find(greenbest))) < 900
        lin_vel = 0.0;
        rot_vel = 0.40;
        
     else % Else track Center of goal 
        
        centgreen = bwlabel(greenbest);
        goalcentroid = regionprops(greenbest,'Centroid');
        Krg = 0.00207;
        
        if (max(size(goalcentroid))) == 1  % If only one object in Binary 
            rot_vel = 0.15;                % Keep rotating
        else               % track ceter of two cylinders
            centerpost1 = goalcentroid(1).Centroid;
            centerpost2 = goalcentroid(2).Centroid;
            centerofgoal = (centerpost1(1) + centerpost2(1)) / 2 ;
            delta = centerofgoal - 320;
        
            rot_vel = -Krg * delta; % P-Controller
        
         if abs(delta) < 13 % if centered
            rot_vel = 0;    % stop
            lin_vel = 0;
            objective1 = 0;
         end
        end
    end        

end

disp(' ')
disp('Done')
disp (' ') 


% Declare Zero frame as bot pointing straight to the goal
resetOdometry(tbot)
% this is our new zero frame


    % Find & Center Orange ball;       
task2 = 1; 
disp('Step 2: Find and center orange ball')
while task2
    
    % obtain image from bot's camera
	rgbImg = getColorImage(tbot);
    % image to hsv format
    hsv = rgb2hsv(rgbImg);
    
    % Orange Ball
    orghue = (hsv(:,:,1) < 0.09); 
    orgsat = (hsv(:,:,2) > 0.65);  
    orgval = (hsv(:,:,3) > 0.5).* (hsv(:,:,3) < 0.999); 
    % combine all matrices to get best binary image 
    orgbin = orghue.* orgsat .* orgval; 
    orgbest = bwareafilt(imbinarize(orgbin),1); %optimal binary image
    
    figure(35)
    imshow(orgbest)

    % Orange ball detect & track (3,000)
    if (length(find(orgbest))) < 550
        lin_vel = 0.0;
        rot_vel = 0.42;
            
    % Else track cylinder
    else 
        % Use Region Properties to find center & area of cylinder
        center = regionprops(orgbest,'centroid');
        ballCenter = center.Centroid;
       
        % Rotational P-Controller
        delta = ballCenter(1) - 320;
        rot_vel = -Kr * delta;
        
        if abs(delta) < 13  % if centered 
            rot_vel = 0;    % stop
            lin_vel = 0;
            task2 = 0;
            
        end
           
    end

end
disp(' ')
disp('Done') 
disp ' '

% This is our andle PHI used to determine position of ball v. goal
angl = curr_pose(3);
Targetangle = wrapToPi(angl); % to keep angle between -pi < phi < pi


    % CONDITION TO SHOOT #1
% if we are lined up, shoot 
if ( abs(Targetangle) < 3.5*(pi/180)) %  3.5 degrees
    disp('Step 3: Lined up, prepare to shoot')
    
    % turn 90 degrees to compensate for sideways camera
    desired_ang = curr_pose(3) + (pi/2);
    
    while ( curr_pose(3) < desired_ang  )
        lin_vel=0.0;
        
        if (curr_pose(3) > 0.70 * desired_ang) 
            rot_vel = pi/22
        else
            rot_vel = pi/9;
        end
        
    end  
    rot_vel = 0.0;
    pause(1)
    
    disp ' '
    disp 'Done' % done with turning
    disp ' '
    
     % Shoot- Straight line
    disp('Step 4: Shoot and score')
   
    resetOdometry(tbot) % reset
    
    while ( curr_pose(1) < 1.40) % distance traveled, 1.4 meters
    Kp = 0.09;                
    delta = wrapToPi(curr_pose(3));    
    lin_vel = 0.49;
    pause(1)
    rot_vel = - Kp * delta;  % Rotational Velocity to counter deviations
    end
    
    lin_vel = 0.0;
    rot_vel = 0.0;
    shoot = 1; % shot has been completed
    
    disp ' ' 
    disp 'Done' 
    disp ' '
    
end

if shoot  % After shooting everything should stop
    pause (2)
    disp('Celebrate')
    % celebrate  
    lin_vel = -0.5; pause(2); lin_vel =0.0;
    rot_vel = 4; pause(3); rot_vel = 0.0;
    
    %Stop All timers and program
    stop(odometry_timer)
    stop(velocity_command_timer)
    stop; 
end


% Line up behind ball & goal

disp('Step 3: Line up behind the ball and center of goal')
objective3 = 1; 
 
position = curr_pose; % will use this to measure distance travelled
while objective3
    
    % obtain image from bot's camera
	rgbImg = getColorImage(tbot);
    % image to hsv format
    hsv = rgb2hsv(rgbImg);
    % Orange Ball
    orghue = (hsv(:,:,1) < 0.09); 
    orgsat = (hsv(:,:,2) > 0.65);  
    orgval = (hsv(:,:,3) > 0.5).* (hsv(:,:,3) < 0.999);  
    % combine all matrices to get best binary image 
    orgbin = orghue.* orgsat .* orgval; 
    orgbest = bwareafilt(imbinarize(orgbin),1); %optimal binary image
    figure(35)
    imshow(orgbest)
    
        % TRACKING ORANGE BALL
    center = regionprops(orgbest,'centroid');
    ballCenter = center.Centroid;   
    delta = ballCenter(1) - 320;
    rot_vel = -Kr * delta; % Rotational P-Controller
   
    
    % Create binary Matrices for each value of HSV FOR GREEN
    greenhue = (hsv(:,:,1) > 0.12).* (hsv(:,:,1) < 0.499); 
    greensat = (hsv(:,:,2) > 0.20).* (hsv(:,:,2) < 0.699);  
    greenval = (hsv(:,:,3) > 0.13).* (hsv(:,:,3) < 0.899);
    greenbin = greenhue.* greensat .* greenval; 
    greenbest = bwareafilt(imbinarize(greenbin),2); 
    % Displays Binary image
    figure(30)
    imshow(greenbest)
    centgreen = bwlabel(greenbest); % Label the  two blobs in binary image
    goalcentroid = regionprops(greenbest,'Centroid');
    if isempty(goalcentroid)
        centerofgoal =  630;
    else
      centerpost1 = goalcentroid(1).Centroid; % centroid of blob #1
      centerpost2 = goalcentroid(2).Centroid; % centroid of blob #2
      centerofgoal = (centerpost1(1) + centerpost2(1)) / 2 ; % Average center
    end
    
    
    % Linear Velocity P-Controller
    
    multiplier = (ballCenter(1) - centerofgoal)/220 ; % signed number
    
    if multiplier < 0
        Kl = -1; % Will dictate the dirention of travel
    else         % either forward or backwards
        Kl = 1;
    end
    
    vel = min(abs(multiplier) , 0.25); % this will cap the velocity at 0.25 or lower
    lin_vel = Kl * vel;
    
   
    if abs(ballCenter(1) - centerofgoal)< 13 % we are lined up
        lin_vel = 0.0;
        rot_vel = 0.0;
        objective3 = 0;
    end
    
    if abs( sqrt(curr_pose(1)^2 + curr_pose(2)^2) - sqrt(position(1)^2 + position(2)^2) ) > 3.5
        failed; % if the robot has traveled more than 3.5 meter away from
                % original position, bot is lost. need to stop
    end
    
end
disp ' '
disp 'Done' %lining up behind ball
disp ' '


% CONDITION TO SHOOT #2
disp('Step 4: Lined up and prepare to shoot')

% turn 90 degrees to compensate for sideways camera
    desired_ang = curr_pose(3) + (pi/2);
    
    while ( curr_pose(3) < desired_ang  )
        lin_vel=0.0;
        
        if (curr_pose(3) > 0.70 * desired_ang) 
            rot_vel = pi/22
        else
            rot_vel = pi/9;
        end
        
    end  
    rot_vel = 0.0;
    pause(1)

 
 disp ' '
 disp 'Done' % turning +90 degrees
 disp ' '
    
    
 % Shoot Straight line
 disp('Step 5: Shoot and score') 
 resetOdometry(tbot) % reset
 
  
     while ( curr_pose(1) < 1.40) % distance traveled, 1.4 meters
    Kp = 0.09;                
    delta = wrapToPi(curr_pose(3));    
    lin_vel = 0.49;
    pause(1)
    rot_vel = - Kp * delta;  % Rotational Velocity to counter deviations
     end

    shoot = 1; % shot has been completed
    disp(' ')

if shoot  % After shooting everything should stop
    pause (2)
    disp('Celebrate')
    % celebrate  
    lin_vel = -0.5; pause(2); lin_vel =0.0;
    rot_vel = 4; pause(3); rot_vel = 0.0;
    
    %Stop All timers and program
    stop(odometry_timer)
    stop(velocity_command_timer)
    stop; 
end
