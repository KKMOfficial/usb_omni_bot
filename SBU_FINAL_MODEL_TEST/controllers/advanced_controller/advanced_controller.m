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

% define and enable gps
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

% define and enable compass
compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP);


% define position sensor variables
wheel_circum = 2 * pi * WHEEL_RADIUS;
encoder_unit = wheel_circum/(2 * pi);



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

  % read compass and rotate arrow accordingly
  compass_val = wb_compass_get_values(compass);

  % read sonar sensors values
  sonar_value = [wb_distance_sensor_get_value(sonar_1) wb_distance_sensor_get_value(sonar_2) wb_distance_sensor_get_value(sonar_3)];
  
  % read position sensors values
  position_value = encoder_unit*[wb_position_sensor_get_value(pos_1),wb_position_sensor_get_value(pos_2),wb_position_sensor_get_value(pos_3)];

  % updating the current theta
  robot_position(3) = atan2(compass_val(1), compass_val(2));
  
  % updating the currnet robot position
  robot_position(1) = gps_values(1);
  robot_position(2) = gps_values(2);
        

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  
  % states here
  if (robot_state == START)
  %%%%%%%%%%%%%%%%%%
    % print("START")
    m_line_angle = calc_m_line(robot_position,goal_postition);
    m_line_point_1 = [goal_postition(1) goal_postition(2)];
    m_line_point_2 = [robot_position(1) robot_position(2)];
    % wall following sensor is the main sensor
    wall_following_sensor = 3;
    if abs(m_line_angle - robot_position(3)) > pi
        robot_omega = [1.0 1.0 1.0];
    else
        robot_omega = [-1.0 -1.0 -1.0];
    end
    robot_state = ROTATE_ROBOT_HEAD_TOWARD_GOAL;
  %%%%%%%%%%%%%%%%%%
  elseif (robot_state == ROTATE_ROBOT_HEAD_TOWARD_GOAL)
  %%%%%%%%%%%%%%%%%%
    % print("ROTATE_ROBOT_HEAD_TOWARD_GOAL")
    if abs(m_line_angle - robot_position(3)) < angle_threshold
        robot_omega = [0.0 0.0 0.0];
        robot_state = MOVE_TOWARD_T_ALONG_M_LINE;
    end          
  %%%%%%%%%%%%%%%%%%
  elseif (robot_state == MOVE_TOWARD_T_ALONG_M_LINE)
  %%%%%%%%%%%%%%%%%%
    % print("MOVE_TOWARD_T_ALONG_M_LINE")
    if  finished(step_timer)
      robot_omega = [0.0 0.0 0.0];
      step_timer = step_timer_reset_value;
      robot_state = SCAN_ENVIRONMENT;
    else
      robot_omega = forward_sensor_direction(wall_following_sensor,1:3);
      step_timer = reduce(step_timer);
    end
  %%%%%%%%%%%%%%%%%%
  elseif (robot_state == SCAN_ENVIRONMENT)
  %%%%%%%%%%%%%%%%%%
    % print("SCAN_ENVIRONMENT")
    if scan_timer == store_pose2
      init_pose2 = robot_position(3);
      scan_timer = reduce(scan_timer);
    elseif scan_timer == scan_left_side
      if abs(init_pose2 - robot_position(3)) < robot_scan_angle 
        robot_omega = [-1.0 -1.0 -1.0];
        % read sensors here
        if is_in_range(sonar_value(wall_following_sensor),hit_threshold_min,hit_threshold_max)
          scan_timer = scan_timer_reset_value;
          wall_following_sensor = wall_following_sensor;
          robot_state = FOLLOW_BOUNDRAY;
        elseif is_in_range(sonar_value(right_sonar(wall_following_sensor)),hit_threshold_min,hit_threshold_max)
          scan_timer = scan_timer_reset_value;
          wall_following_sensor = right_sonar(wall_following_sensor);
          robot_state = FOLLOW_BOUNDRAY;
        end    
      else
        robot_omega = [0.0 0.0 0.0];
        scan_timer = reduce(scan_timer);
      end
    elseif scan_timer == reset_head_dir
      if abs(init_pose2 - robot_position(3)) > angle_threshold
        robot_omega = [1.0 1.0 1.0];
      else
        robot_omega = [0.0 0.0 0.0];
        scan_timer = reduce(scan_timer);
       end    
    elseif scan_timer == scan_right_side
      if abs(init_pose2 - robot_position(3)) < robot_scan_angle
        robot_omega = [1.0,1.0,1.0];
        % read sensors here
        if is_in_range(sonar_value(wall_following_sensor),hit_threshold_min,hit_threshold_max)
          scan_timer = scan_timer_reset_value;
          wall_following_sensor = wall_following_sensor;
          robot_state = FOLLOW_BOUNDRAY;
        elseif is_in_range(sonar_value(right_sonar(wall_following_sensor)),hit_threshold_min,hit_threshold_max)
          scan_timer = scan_timer_reset_value;
          wall_following_sensor = right_sonar(wall_following_sensor);
          robot_state = FOLLOW_BOUNDRAY;
        end    
      else
        robot_omega = [0.0 0.0 0.0];
        scan_timer = reduce(scan_timer);
      end
    elseif finished(scan_timer)
      if abs(init_pose2 - robot_position(3)) > angle_threshold
        robot_omega = [-1.0 -1.0 -1.0];
      else
        robot_omega = [0.0 0.0 0.0];
        scan_timer = scan_timer_reset_value;
        robot_state = MOVE_TOWARD_T_ALONG_M_LINE;
      end
    end
  %%%%%%%%%%%%%%%%%%
  elseif (robot_state == FOLLOW_BOUNDRAY)
  %%%%%%%%%%%%%%%%%%
    if finished(follow_wall_timer)
      %wb_console_print('finished', WB_STDOUT);
      follow_wall_timer = follow_wall_timer_reset_value;
      robot_state = EDGE_DETECTION;
      scan_timer = scan_timer_reset_value;
      circular_move_timer = circular_move_timer_reset_value;
      % one_step_estimated_displacement = np.sum(curr_accumulated_estimated_displacement)
      % print("ONE : ",one_step_estimated_displacement)
      % print("left: ",left_move_counter)
      % print("right: ",right_move_counter)
      % print("------------")
      [left_move_counter,right_move_counter] = deal(0,0);
      % curr_accumulated_estimated_displacement = 0.0
        
    else
      follow_wall_timer = reduce(follow_wall_timer);
      if (abs(left_move_counter + right_move_counter -follow_wall_timer_reset_value) < bot_stop_threshold)
        %wb_console_print('circular move', WB_STDOUT);
        if(circular_move_timer > 3000)
          follow_wall_timer = follow_wall_timer + 1;
          circular_move_timer = reduce(circular_move_timer);
          robot_omega = -0.4*forward_sensor_direction(wall_following_sensor,1:3);
        elseif(circular_move_timer <= 3000) & (circular_move_timer > 1)
          follow_wall_timer = follow_wall_timer + 1;
          circular_move_timer = reduce(circular_move_timer);
          robot_omega = circular_movement_counter_clockwise(wall_following_sensor,1:3);
          % prevend hitting the wall
          if is_in_range(sonar_value(right_sonar(wall_following_sensor)),hit_threshold_min,hit_threshold_max)
              robot_omega = [0.0 0.0 0.0];
              circular_move_timer = 1;
          end    
        elseif(circular_move_timer == 1)
          circular_move_timer = circular_move_timer_reset_value;
          wall_following_sensor = right_sonar(wall_following_sensor);
          [left_move_counter,right_move_counter] = deal(0,0);
          follow_wall_timer = follow_wall_timer_reset_value;
          scan_timer = scan_timer_reset_value;
          circular_move_timer = circular_move_timer_reset_value;
          step_timer = step_timer_reset_value;
        end    
      elseif exceeds_upper_limit(sonar_value(wall_following_sensor),hit_threshold_max)
        %wb_console_print('getting away', WB_STDOUT);
        robot_omega = 2*rotational_movement_counter_clockwise(wall_following_sensor,1:3);
        left_move_counter = left_move_counter + 1;
      elseif exceeds_lower_limit(sonar_value(wall_following_sensor),hit_threshold_min)
        %wb_console_print('getting near', WB_STDOUT);
        robot_omega = 2*rotational_movement_clockwise(wall_following_sensor,1:3);
        right_move_counter = right_move_counter + 1;
      else
        % robot_omega = forward_sensor_direction[wall_following_sensor]
        %wb_console_print('parallel move', WB_STDOUT);     
        robot_omega = 0.5*parallel_move(right_motor(wall_following_sensor),1:3) + 0.5*forward_sensor_direction(wall_following_sensor,1:3);
      end
    end       
  %%%%%%%%%%%%%%%%%%              
  elseif (robot_state == EDGE_DETECTION)
  %%%%%%%%%%%%%%%%%%
    % print("SCAN_ENVIRONMENT")
    if scan_timer == store_pose2
      init_pose2 = robot_position(3);
      scan_timer = reduce(scan_timer);
    elseif scan_timer == scan_left_side
      if abs(init_pose2 - robot_position(3)) < robot_scan_angle
        robot_omega = [-1.0 -1.0 -1.0];
        % read sensors here
        if is_in_range(sonar_value(right_sonar(wall_following_sensor)),hit_threshold_min,hit_threshold_max)
            scan_timer = scan_timer_reset_value;
            wall_following_sensor = right_sonar(wall_following_sensor);
            robot_state = FOLLOW_BOUNDRAY;
        end    
      else
        robot_omega = [0.0 0.0 0.0];
        scan_timer = reduce(scan_timer);
      end    
    elseif scan_timer == reset_head_dir
      if abs(init_pose2 - robot_position(3)) > angle_threshold
        robot_omega = [1.0 1.0 1.0];
      else
        robot_omega = [0.0 0.0 0.0];
        scan_timer = reduce(scan_timer);
      end    
    elseif scan_timer == scan_right_side
      if abs(init_pose2 - robot_position(3)) < robot_scan_angle
        robot_omega = [1.0 1.0 1.0];
        % read sensors here
        if is_in_range(sonar_value(right_sonar(wall_following_sensor)),hit_threshold_min,hit_threshold_max)
            scan_timer = scan_timer_reset_value;
            wall_following_sensor = right_sonar(wall_following_sensor);
            robot_state = FOLLOW_BOUNDRAY;
        end    
      else
        robot_omega = [0.0 0.0 0.0];
        scan_timer = reduce(scan_timer);
      end    
    elseif finished(scan_timer)
      if abs(init_pose2 - robot_position(3)) > angle_threshold
        robot_omega = [-1.0 -1.0 -1.0];
      else
        robot_omega = [0.0 0.0 0.0];
        scan_timer = scan_timer_reset_value;
        robot_state = FOLLOW_BOUNDRAY;
        follow_wall_timer = follow_wall_timer_reset_value;
      end
    end        
  end
  %%%%%%%%%%%%%%%%%%                    
  % elseif (robot_state == SWAP_FOLLOWING_WALL):
  %%%%%%%%%%%%%%%%%%
      % robot_omega = [0.0 0.0 0.0])
  %%%%%%%%%%%%%%%%%%
  %elseif (robot_state == SWAP_ACTIVE_SONAR):

  %elseif (robot_state == STOP):

  % elseif (robot_state == TEST):
      % robot_omega = [0 0 0]
  
  
  
  
  
  
  % update motor velocities
  % add noise to motor velocity
  noise = normrnd(0,0.15,[1,3]);
  wb_motor_set_velocity(motor_1, robot_omega(1) + noise(1));
  wb_motor_set_velocity(motor_2, robot_omega(2) + noise(2));
  wb_motor_set_velocity(motor_3, robot_omega(3) + noise(3));
  
  % estimate robot movement
  estimate_displacement = WHEEL_RADIUS * kinematic(WHEEL_RADIUS, robot_position(3), robot_omega,alpha1,alpha2,alpha3,CHASSIS_AXLE_LENGTH);  
  
  % measure robot movement
  robot_phi_measure = position_value - robot_phi_measure;
  measure_phi = robot_phi_measure / WHEEL_RADIUS;
  estimate_displacement = WHEEL_RADIUS*kinematic(WHEEL_RADIUS, robot_position(3), measure_phi,alpha1,alpha2,alpha3,CHASSIS_AXLE_LENGTH);
  robot_pos_measure(1) = robot_pos_measure(1) + estimate_displacement(1);
  robot_pos_measure(2) = robot_pos_measure(2) + estimate_displacement(2);
  
  
  

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

