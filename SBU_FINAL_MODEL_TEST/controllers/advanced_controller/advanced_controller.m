% MATLAB controller for Webots
% File:          advanced_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;
WHEEL_RADIUS = 0.029;
CHASSIS_AXLE_LENGTH = 0.22;


% see the report(appendix A) to understand how alpha angle is defined.
alpha1 = deg2rad(30);
alpha2 = deg2rad(150);
alpha3 = deg2rad(270);


% define timers and corresponding reset values here
% used for step movements toward goal; after each step robot will scan the environment
[step_timer, step_timer_reset_value] = deal(60, 60);
% used for detecting wall and scan process
[scan_timer, scan_timer_reset_value] = deal(4, 4);
store_pose2 = 4;
scan_left_side = 3;
reset_head_dir = 2;
scan_right_side = 1;
% after each wall following step, the robot will check it's envirionment
[follow_wall_timer, follow_wall_timer_reset_value] = deal(60, 60);
% circulate timer
[circular_move_timer, circular_move_timer_reset_value] = deal(3200, 3200);

% define thresholds here
% angle between heading and target
angle_threshold = 0.01;
% for distance from M-line
m_line_threshold = 0.1;
% for obstacle hit detection
hit_threshold_min = 25;
hit_threshold_max = 300;
% scan angle
robot_scan_angle = pi/2;
% robot stop threshold
bot_stop_threshold = 10;


% define special memories here; these values will be use during the procedure
% will store initial value of the head angle
init_pose2 = 0;
% main sensor
wall_following_sensor = 1;
% estimate displacement using position sensor travesed (in one wall follow step mainly)
curr_accumulated_estimated_displacement = 0.0;
one_step_estimated_displacement = 0.0;
% variables for storing position information
current_position = [0.0 0.0 0.0];
previous_position = [0.0 0.0 0.0];
% this vector holds the prev value of the position sensor
last_ps_values = [0.0 0.0 0.0];
% these variable will store control direction counters
right_move_counter = 0;
left_move_counter = 0;
% this variable used to store scan data into an array
scans = [];
% used to control lidar sampling store rate
sample_enable = 0;
% used to show real trajectory of the bot
real_x = [];
real_y = [];
% used to declare occupancy map has been created and localization started
planning_mode = false;
localization_mode = false;
slam_mode = true;
% localization algorithm variables
odometryPose = [0 0 0];
loc_counter = 0;
% planning algorithm variables
%occGrid

% define algorithm states here

% for debugging purposes.
TEST = 0;
% extracting M-line.
START = 1;
% rotate the heading towarad_goal.
ROTATE_ROBOT_HEAD_TOWARD_GOAL = 2;
% bang bang movement in straight line.
MOVE_TOWARD_T_ALONG_M_LINE = 3;
% will rotate in both directions and measures it's sonars values to detect wall
SCAN_ENVIRONMENT = 8;
% adjust the heading tangent to obstacle.
ROTATE_ROBOT_HEAD_PARALLEL_TO_OBSTACLE = 4;
% follow wall.
FOLLOW_BOUNDRAY = 5;
% detect edges while following wall
EDGE_DETECTION = 9;
% swap and update wall following sensor and the followed wall itself
SWAP_FOLLOWING_WALL = 10;
% one step to wall distance control
ONE_STEP_ADJUSMENT = 11;
% related to detecting a corner.
SWAP_ACTIVE_SONAR = 6;
% last state of the algorithm
STOP = 7;

% specific velocity directions
forward_sensor_direction = [0.0 3.0 -3.0;
                            -3.0,0.0,3.0;
                            3.0 -3.0 0.0];

rotational_movement_clockwise = [3.0 0.0 0.0;
                                 0.0 3.0 0.0;
                                 0.0 0.0 3.0];
                                 
                                       
rotational_movement_counter_clockwise = [-3.0 0.0 0.0;
                                         0.0 -3.0 0.0;
                                         0.0 0.0 -3.0];
                                         
                                       
circular_movement_clockwise = [-3.0 3.0 3.0;
                               3.0 -3.0 3.0;
                               3.0 3.0 -3.0];
                              
                                       
circular_movement_counter_clockwise = [0.85 -1.0 -1.0;
                                       -1.0 0.85 -1.0;
                                       -1.0 -1.0 0.85];
                                       
                                               
                                               
parallel_move = [-6.0 3.0 3.0;
                 3.0 -6.0 3.0;
                 3.0 3.0 -6.0];
                

% define robot state here
robot_velocity = [0.0 0.0 0.0];
robot_position = [-14.28 -16.6 0.0];
robot_omega    = [0.0, 0.0, 0.0];
robot_state    = START;
robot_pos_estimate = robot_position(1:3);
robot_pos_measure = robot_position(1:3);
robot_phi_measure = [0.0 0.0 0.0];



% define target state here
goal_postition = [15.47 16.57];

% define m_line properties
m_line_angle   = 0.0;
m_line_point_1 = goal_postition(1:2); 
m_line_point_2 = robot_position(1:2); 
robot_distance_from_m_line = 0.0;


% define robot motors
motor_1 = wb_robot_get_device('joint_right_wheel');
motor_2 = wb_robot_get_device('joint_left_wheel');
motor_3 = wb_robot_get_device('joint_left2_wheel');  

% set position for robot motors
wb_motor_set_position(motor_1, inf);
wb_motor_set_position(motor_2, inf);
wb_motor_set_position(motor_3, inf);

% set velocity for robot motors
wb_motor_set_velocity(motor_1, 0.0);
wb_motor_set_velocity(motor_2, 0.0);
wb_motor_set_velocity(motor_3, 0.0);

% define distance sensors
sonar_1 = wb_robot_get_device('sonar_1');
sonar_2 = wb_robot_get_device('sonar_2');
sonar_3 = wb_robot_get_device('sonar_3');

% enable distance sensors
wb_distance_sensor_enable(sonar_1, TIME_STEP);
wb_distance_sensor_enable(sonar_2, TIME_STEP);
wb_distance_sensor_enable(sonar_3, TIME_STEP);

% define IR sensors
ir_1 = wb_robot_get_device('distance_sensor1');
ir_2 = wb_robot_get_device('distance_sensor2');
ir_3 = wb_robot_get_device('distance_sensor3');
ir_4 = wb_robot_get_device('distance_sensor4');
ir_5 = wb_robot_get_device('distance_sensor5');
ir_6 = wb_robot_get_device('distance_sensor6');

% enable IR sensors
wb_distance_sensor_enable(ir_1, TIME_STEP);
wb_distance_sensor_enable(ir_2, TIME_STEP);
wb_distance_sensor_enable(ir_3, TIME_STEP);
wb_distance_sensor_enable(ir_4, TIME_STEP);
wb_distance_sensor_enable(ir_5, TIME_STEP);
wb_distance_sensor_enable(ir_6, TIME_STEP);


% define position sensors
pos_1 = wb_robot_get_device('joint_right_wheel_sensor');
pos_2 = wb_robot_get_device('joint_left_wheel_sensor');
pos_3 = wb_robot_get_device('joint_left2_wheel_sensor');  

% enable position sensors
wb_position_sensor_enable(pos_1, TIME_STEP);
wb_position_sensor_enable(pos_2, TIME_STEP);
wb_position_sensor_enable(pos_3, TIME_STEP);

wheel_cirum = 2 * pi * WHEEL_RADIUS;
encoder_unit = wheel_cirum / (2*pi);

% define and enable lidar sensors
lidar = wb_robot_get_device('head_hokuyo_sensor');
wb_lidar_enable(lidar, TIME_STEP);
wb_lidar_enable_point_cloud(lidar);
num_of_points = wb_lidar_get_number_of_points(lidar);


% defien and enable noisy gps for odometry
odometry = wb_robot_get_device('odometry');
wb_gps_enable(odometry, TIME_STEP);

% define and enable gps
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

% define and enable compass
compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP);


% define position sensor variables
wheel_circum = 2 * pi * WHEEL_RADIUS;
encoder_unit = wheel_circum/(2 * pi);

% enable keyboard
wb_keyboard_enable(32);


% define lidar sensor required data
% Set maximum lidar range to be slightly smaller than maximum range of the
% scanner, as the laser readings are less accurate near maximum range
maxLidarRange = wb_lidar_get_max_range(lidar);
minLidarRange = wb_lidar_get_min_range(lidar);
% Set the map resolution to 10 cells per meter, which gives a precision of
% 10cm
mapResolution = 10;
% Create a pose graph object and define information matrix
pGraph = poseGraph;
infoMat = [1 0 0 1 0 1];
% boolean flag to identify the first scan
is_first_scan = true;
loopClosureThreshold = 110;
loopClosureSearchRadius = 2;


% simulation mode
remote_control_mode = false;
wall_following_mode = false;


prompt = 'enter [r] for remote control mode, [w] for wall following mode.';
mode = input(prompt, 's')
if mode == 'r'
  remote_control_mode = true;
  wb_console_print('use [A],[S],[D],[W] to move, [R] to clock-wise rotation, [C] to counter clock-wise rotation.', WB_STDOUT);
  wb_console_print('use [T] to terminate process and generate map.', WB_STDOUT);
  
else
  wall_following_mode = true;
end





% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%

while wb_robot_step(TIME_STEP) ~= -1

  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);
  % read GPS values
  gps_values = wb_gps_get_values(gps);
  
  % read GPS values for odometry
  odometry_values = wb_gps_get_values(odometry);

  % read compass and rotate arrow accordingly
  compass_val = wb_compass_get_values(compass);

  % read sonar sensors values
  sonar_value = [wb_distance_sensor_get_value(sonar_1) wb_distance_sensor_get_value(sonar_2) wb_distance_sensor_get_value(sonar_3)];
  
  % read ir sensors values
  ir_value = [wb_distance_sensor_get_value(ir_1) ... 
              wb_distance_sensor_get_value(ir_2) ... 
              wb_distance_sensor_get_value(ir_3) ... 
              wb_distance_sensor_get_value(ir_4) ... 
              wb_distance_sensor_get_value(ir_5) ... 
              wb_distance_sensor_get_value(ir_6)];
  
  % read position sensors values
  position_value = encoder_unit*[wb_position_sensor_get_value(pos_1),wb_position_sensor_get_value(pos_2),wb_position_sensor_get_value(pos_3)];

  % updating the current theta
  robot_position(3) = atan2(compass_val(1), compass_val(2));
  
  % updating the currnet robot position
  robot_position(1) = gps_values(1);
  robot_position(2) = gps_values(2);
  
  % updating lidar scan data 
  lidar_points_set = wb_lidar_get_range_image(lidar);
  
  if(wall_following_mode)

        
    % update motor velocities
    % add noise to motor velocity
    noise = normrnd(0,0.15,[1,3]);
    wb_motor_set_velocity(motor_1, robot_omega(1) + noise(1));
    wb_motor_set_velocity(motor_2, robot_omega(2) + noise(2));
    wb_motor_set_velocity(motor_3, robot_omega(3) + noise(3));     
       
  elseif (remote_control_mode)
    % get input with tic toc
    ch = wb_keyboard_get_key();
    
    % proces input data
    if ch == -1
      robot_omega = [0.0 0.0 0.0];
    else
      if ch == 65 %'a'
        robot_omega = [3.0 3.0 -6.0];
        sample_enable = sample_enable + 1;
      elseif ch == 83 %'s'
        robot_omega = [-3.0 3.0 0.0];      
        sample_enable = sample_enable + 1;        
      elseif ch == 68 %'d'
        robot_omega = [-3.0 -3.0 6.0];  
        sample_enable = sample_enable + 1;      
      elseif ch == 87 %'w'
        robot_omega = [3.0 -3.0 0.0];
        sample_enable = sample_enable + 1;     
      elseif ch == 82 %'r'
        robot_omega = [-3.0 -3.0 -3.0];
      elseif ch == 67 %'c'
        robot_omega = [3.0 3.0 3.0];     
      elseif ch == 76 %'l'  
        localization_mode = true;     
        fprintf('localization mode enabled successfully \n');
      elseif ch == 80 %'p'
        localization_mode = false;
      
        fprintf('planning mode enabled successfully \n');
        show(occGrid)
        
        x_g = input('goal X = ');
        y_g = input('goal Y = ');
        
        
        
        % Set the start and goal poses
        start = robot_position(:,1:3);
        goal = [x_g, y_g, 0];
        
        % Show the start and goal positions of the robot
        hold on
        plot(start(1), start(2), 'ro')
        plot(goal(1), goal(2), 'mo')
        
        % Show the start and goal headings
        r = 0.5;
        plot([start(1), start(1) + r*cos(start(3))], [start(2), start(2) + r*sin(start(3))], 'r-' )
        plot([goal(1), goal(1) + r*cos(goal(3))], [goal(2), goal(2) + r*sin(goal(3))], 'm-' )
        hold off
        
        bounds = [occGrid.XWorldLimits; occGrid.YWorldLimits; [-pi pi]];

        ss = stateSpaceDubins(bounds);
        ss.MinTurningRadius = 0.4;
        
        
        stateValidator = validatorOccupancyMap(ss); 
        stateValidator.Map = occGrid;
        stateValidator.ValidationDistance = 0.05;
        
        
        planner = plannerRRT(ss, stateValidator);
        planner.MaxConnectionDistance = 2.0;
        planner.MaxIterations = 30000;
        
        planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
        
        rng(0,'twister')

        [pthObj, solnInfo] = plan(planner, start, goal);
        
        show(occGrid)
        hold on
        
        % Search tree
        plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');
        
        % Interpolate and plot path
        interpolate(pthObj,300)
        plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)
        
        % Show the start and goal in the grid map
        plot(start(1), start(2), 'ro')
        plot(goal(1), goal(2), 'mo')
        hold off

      elseif slam_mode & (ch == 84) %'t'
        robot_omega = [0.0 0.0 0.0];
        wb_motor_set_velocity(motor_1, 3*robot_omega(1));
        wb_motor_set_velocity(motor_2, 3*robot_omega(2));
        wb_motor_set_velocity(motor_3, 3*robot_omega(3));
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        maxLidarRange = 16;
        mapResolution = 10;
        slamAlg = lidarSLAM(mapResolution, maxLidarRange);
        
        slamAlg.LoopClosureThreshold = 360;  
        slamAlg.LoopClosureSearchRadius = 8;
         
        
        for i=1:length(scans)
          [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans(i));
          if isScanAccepted
            fprintf('Added scan %d \n', i);
          end
        end  
        
                
        firstTimeLCDetected = false;

        figure;
        for i=1:length(scans)
          [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans(i));
          if ~isScanAccepted
              continue;
          end
          
          % visualize the first detected loop closure, if you want to see the
          % complete map building process, remove the if condition below
          if optimizationInfo.IsPerformed && ~firstTimeLCDetected
          show(slamAlg, 'Poses', 'off');
          hold on;
          show(slamAlg.PoseGraph); 
          hold off;
          firstTimeLCDetected = true;
          drawnow
          end
        end
        % plot(real_x,real_y)
        
        title('First loop closure');
        
        
        
        [scans, optimizedPoses]  = scansAndPoses(slamAlg);
        map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
        
        
        figure; 
        show(map);
        hold on
        show(slamAlg.PoseGraph, 'IDs', 'off');
        hold off
        title('Occupancy Grid Map Built Using Lidar SLAM');
        
        
        % this variable is going to be used in path planning phase
        occGrid = copy(map);

        
        fprintf('finished \n');
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        fprintf('robot will move toward the goal. \n');
        % enable particle filter to initiate localization phase
        
        % use odometryModel for differential drive
        % odometryModel = odometryMotionModel;
        % odometryModel.Noise = [0.2 0.2 0.2 0.2];
        
        odometryPose = [odometry_values(1) odometry_values(2) robot_position(3)];
        
        rangeFinderModel = likelihoodFieldSensorModel;
        rangeFinderModel.SensorLimits = [0.01 16];
        rangeFinderModel.Map = map;
        
        amcl = monteCarloLocalization;
        amcl.UseLidarScan = true;
        
        amcl.SensorModel = rangeFinderModel;
        
        amcl.UpdateThresholds = [0.2,0.2,0.2];
        amcl.ResamplingInterval = 1;
        
        amcl.ParticleLimits = [500 5000];
        amcl.GlobalLocalization = false;
        amcl.InitialPose = robot_position(:,1:3);
        amcl.InitialCovariance = eye(3)*0.5;
        
        
        visualizationHelper = ExampleHelperAMCLVisualization(map)
        
        % figure;
        % drawnow;
        % title('Localization');
        
        
        % VFH = robotics.VectorFieldHistogram;
        % VFH.DistanceLimits = [0 2];
        % VFH.RobotRadius = 0.12;
        % VFH.MinTurningRadius = 0.12;
        % VFH.SafetyDistance = 0.05;
        %% obj.vfh.HistogramThresholds= [3 10];
        % TargetDir = 0;
        
        slam_mode = false;
        localization_mode = false;
        planning_mode = false;
            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        
      else
        robot_omega = [0.0 0.0 0.0];
      end
      
      if sample_enable == 20 & slam_mode
        scans = [scans convert_to_lidar_scan(lidar_points_set,robot_position(3))];
        real_x = [real_x robot_position(1)];
        real_y = [real_y robot_position(2)];         
        sample_enable = 0;
      end
      
      
      if sample_enable == 20 & localization_mode
        scans = convert_to_lidar_scan(lidar_points_set,robot_position(3));
        real_x = robot_position(1);
        real_y = robot_position(2);
        sample_enable = 0;
        
        % update position with each movement and graph it
        [isUpdated,estimatedPose,covariance] = amcl(odometryPose,scans);
        
        % update odometry position
        odometryPose = estimatedPose;
        
        % Call VFH object to computer steering direction
        % steerDir = VFH(scan, targetDir); 

        
        % figure; 
        % hold on;
        plotStep(visualizationHelper, amcl, estimatedPose, scans, loc_counter)
        %plot(robot_position(1),robot_position(2),'r-o');
        % hold off;
        % drawnow;
        % title('Localization');
        
        loc_counter = loc_counter + 1;
      end
      

      
    end
    
    wb_motor_set_velocity(motor_1, 3*robot_omega(1));
    wb_motor_set_velocity(motor_2, 3*robot_omega(2));
    wb_motor_set_velocity(motor_3, 3*robot_omega(3));
  end
  
  
  
  

  

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
% the rolling constraint matrix from each wheel. (read the report for more detail)


function J1 = J1_matrix(theta,alpha1,alpha2,alpha3,CHASSIS_AXLE_LENGTH)
  J1 = [cos(alpha1+theta) sin(alpha1+theta) -CHASSIS_AXLE_LENGTH;
        cos(alpha2+theta) sin(alpha2+theta) -CHASSIS_AXLE_LENGTH;
        cos(alpha3+theta) sin(alpha3+theta) -CHASSIS_AXLE_LENGTH]; 
end


function Velocity_Vector = kinematic(WHEEL_RADIUS, robot_theta, phi_dot,alpha1,alpha2,alpha3,CHASSIS_AXLE_LENGTH)
  Velocity_Vector = WHEEL_RADIUS * (inv(J1_matrix(robot_theta,alpha1,alpha2,alpha3,CHASSIS_AXLE_LENGTH)) * transpose(phi_dot));
end


function Distance_From_Line = Calculate_Point_Distance(l1, l2, p3)
  Distance_From_Line = norm(cross(l2-l1, l1-p3))/norm(l2-l1);
end


function M_Line = calc_m_line(robot_pos, goal_pos)
  M_Line = (robot_pos(2)-goal_pos(2)) / (robot_pos(1)-goal_pos(1));
end

function sonar_num = right_sonar(active_sonar)
  sonar_num = mod(active_sonar-2,3)+1;
end


function sonar_num = left_sonar(active_sonar)
  sonar_num = mod(active_sonar,3)+1;
end


function motor_num = right_motor(active_sonar)
  motor_num = mod(active_sonar-2,3)+1;
end


function motor_num = left_motor(active_sonar)
  motor_num = mod(active_sonar,3)+1;
end


function motor_num = active_motor(active_sonar)
  motor_num = active_sonar;
end


function is_finished = finished(timer_value)
  is_finished = timer_value <= 0;
end


function reduced_value = reduce(timer_value)
  reduced_value = timer_value-1;
end

function result = is_in_range(sonar_value, min_value, max_value)
  result = (sonar_value >= min_value) & (sonar_value <= max_value);
end


function result = exceeds_upper_limit(sonar_value,max_value)
  result = sonar_value > max_value;
end


function result = exceeds_lower_limit(sonar_value,min_value)
  result = sonar_value < min_value;
end

function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
  isReached = false;
  threshold = 0.1;
  if planner.StateSpace.distance(newState, goalState) < threshold
      isReached = true;
  end
end

function lidarScanVal = convert_to_lidar_scan(lidar_img_points,robot_theta)
  % class(lidar_img_points)
  % numel(lidar_img_points)
  % lidar_img_points
  % pause(300)
  refAngles  = double(linspace(0,-2*pi,numel(lidar_img_points)));
  refRanges  = double(lidar_img_points);
  lidarScanVal = lidarScan(refRanges,refAngles);
end

