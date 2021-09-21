"""move_bot_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import GPS
from controller import Compass
from controller import DistanceSensor
from controller import PositionSensor
from controller import LidarPoint
from controller import Display
from matplotlib.animation import FuncAnimation
from threading import Thread

import math
import numpy as np
import matplotlib.pyplot as plt
from numpy import linalg as la
from multiprocessing import Process

# the units are yet to be determined. But the scale is correct and it is in milimeters IRL.
WHEEL_RADIUS = 0.029 
# AKA r
CHASSIS_AXLE_LENGTH = 0.22 
# AKA l
# fig = plt.figure()
# ax = fig.add_subplot(1, 1, 1)




# see the report(appendix A) to understand how alpha angle is defined.
alpha1 = math.radians(30)
alpha2 = math.radians(150)
alpha3 = math.radians(270)

# the rolling constraint matrix from each wheel. (read the report for more detail)
def J1_matrix(theta):
    return np.array([[math.cos(alpha1+theta), math.sin(alpha1+theta), -CHASSIS_AXLE_LENGTH],
                     [math.cos(alpha2+theta), math.sin(alpha2+theta), -CHASSIS_AXLE_LENGTH],
                     [math.cos(alpha3+theta), math.sin(alpha3+theta), -CHASSIS_AXLE_LENGTH]])

def rotation_matrix(theta):
    return np.array([[math.cos(theta), -math.sin(theta),     0   ],
                     [math.sin(theta),  math.cos(theta),     0   ],
                     [       0       ,         0      ,      1   ]])
        
                     
# def inverse_kinematic(wheel_radius, theta, xi_dot):
    # R = rotation_matrix(theta)
    # phi_dot = (1/wheel_radius) * np.matmul(np.matmul(J1,R),xi_dot)
    # return phi_dot
    
def kinematic(robot_theta, phi_dot):
    return WHEEL_RADIUS * np.matmul(np.linalg.inv(J1_matrix(robot_theta)),phi_dot)

def calc_destance_from_line(l1, l2, p3):
    return la.norm(np.cross(l2-l1, l1-p3))/la.norm(l2-l1)

def calc_m_line(r_pos, g_pos):
    join  = np.vstack((r_pos[0:2],g_pos)).transpose()
    delta = np.matmul(join, [[-1],[1]])
    return math.atan(delta[1]/delta[0])
    
def right_sonar(active_sonar):
    return (active_sonar-1)%3
    
def left_sonar(active_sonar):
    return (active_sonar+1)%3
    
def right_motor(active_sonar):
    return (active_sonar-1)%3
    
def left_motor(active_sonar):
    return (active_sonar+1)%3
    
def active_motor(active_sonar):
    return active_sonar
    
def finished(timer_value):    
    return timer_value <= 0
    
def reduce(timer_value):
    return timer_value-1
    
def is_in_range(sonar_value, min_value, max_value):
    return (sonar_value >= min_value) & (sonar_value <= max_value)
    
def exceeds_upper_limit(sonar_value,max_value):
    return sonar_value > max_value
    
def exceeds_lower_limit(sonar_value,min_value):
    return sonar_value < min_value

# def animate(i):
    # global data
    # data = data[-20:]
    # ax.clear()
    # for point in data:
        # ax.scatter(point[0],point[1],color=point[2])
    # plotting using multiprocessing liberary
    # plt.xlim(-40.0,40.0,1.0)
    # plt.ylim(-40.0,40.0,1.0)
    # plt.title('Extended Kalman Filter Plot')
    


# used in controller
global move_turn

# global data
move_turn = 0
    
    
if __name__ == "__main__":
    # data = []
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = 32
        
    # define timers and corresponding reset values here
    # used for step movements toward goal; after each step robot will scan the environment
    step_timer, step_timer_reset_value = 120, 120
    # used for detecting wall and scan process
    scan_timer, scan_timer_reset_value = 4, 4
    store_pose2 = 4
    scan_left_side = 3
    reset_head_dir = 2
    scan_right_side = 1
    # after each wall following step, the robot will check it's envirionment
    follow_wall_timer, follow_wall_timer_reset_value = 120, 120
    # circulate timer
    circular_move_timer, circular_move_timer_reset_value = 3200, 3200
    
    # define thresholds here
    # angle between heading and target
    angle_threshold = 0.0025
    # for distance from M-line
    m_line_threshold = 0.1
    # for obstacle hit detection
    hit_threshold_min = 100#200
    hit_threshold_max = 300#400
    # scan angle
    robot_scan_angle = math.pi/2  
    # robot stop threshold
    bot_stop_threshold = 10
    
    
    # define special memories here; these values will be use during the procedure
    # will store initial value of the head angle
    init_pose2 = 0
    # main sensor
    wall_following_sensor = 0
    # estimate displacement using position sensor travesed (in one wall follow step mainly)
    curr_accumulated_estimated_displacement = 0.0
    one_step_estimated_displacement = 0.0
    # variables for storing position information
    current_position = np.array([0.0,0.0,0.0])
    previous_position = np.array([0.0,0.0,0.0])
    # this vector holds the prev value of the position sensor
    last_ps_values = np.array([0.0,0.0,0.0])
    # these variable will store control direction counters
    right_move_counter = 0
    left_move_counter = 0
    
    
    # define algorithm states here
    
    # for debugging purposes.
    TEST = 0
    # extracting M-line.
    START = 1
    # rotate the heading towarad_goal.
    ROTATE_ROBOT_HEAD_TOWARD_GOAL = 2
    # bang bang movement in straight line.
    MOVE_TOWARD_T_ALONG_M_LINE = 3
    # will rotate in both directions and measures it's sonars values to detect wall
    SCAN_ENVIRONMENT = 8
    # adjust the heading tangent to obstacle.
    ROTATE_ROBOT_HEAD_PARALLEL_TO_OBSTACLE = 4
    # follow wall.
    FOLLOW_BOUNDRAY = 5
    # detect edges while following wall
    EDGE_DETECTION = 9
    # swap and update wall following sensor and the followed wall itself
    SWAP_FOLLOWING_WALL = 10
    # one step to wall distance control
    ONE_STEP_ADJUSMENT = 11
    # related to detecting a corner.
    SWAP_ACTIVE_SONAR = 6
    STOP = 7

    # specific velocity directions
    forward_sensor_direction = np.array([[3.0,-3.0,0.0]
                                        ,[0.0,3.0,-3.0]
                                        ,[-3.0,0.0,3.0]])
    
    rotational_movement_clockwise = np.array([[0.0,0.0,3.0]
                                             ,[3.0,0.0,0.0]
                                             ,[0.0,3.0,0.0]])
                                           
    rotational_movement_counter_clockwise = np.array([[0.0,0.0,-3.0]
                                                     ,[-3.0,0.0,0.0]
                                                     ,[0.0,-3.0,0.0]])  
                                           
    circular_movement_clockwise = np.array([[3.0,3.0,-3.0]
                                           ,[-3.0,3.0,3.0]
                                           ,[3.0,-3.0,3.0]])        
                                           
    circular_movement_counter_clockwise = np.array([[-1.0,-1.0,0.85]
                                                   ,[0.85,-1.0,-1.0]
                                                   ,[-1.0,0.85,-1.0]])  
                                                   
                                                   
    parallel_move = np.array([[3.0,3.0,-6.0]
                               ,[-6.0,3.0,3.0]
                               ,[3.0,-6.0,3.0]])                                 

    # define robot state here
    robot_velocity = np.array([0.0, 0.0, 0.0])
    robot_position = np.array([-14.28, -16.6, 0.0])
    robot_omega    = np.array([0.0, 0.0, 0.0])
    robot_state    = START
    robot_pos_estimate = np.array([robot_position[0],robot_position[1],robot_position[2]])
    robot_pos_measure = np.array([robot_position[0],robot_position[1],robot_position[2]])
    robot_phi_measure = np.array([0.0,0.0,0.0])
    

    
    # define target state here
    goal_postition = np.array([15.47,16.57])
    
    # define m_line properties
    m_line_angle   = 0.0
    m_line_point_1 = None 
    m_line_point_2 = None 
    robot_distance_from_m_line = 0.0
    
    # define robot motors
    motor_1 = robot.getDevice("joint_right_wheel")
    motor_2 = robot.getDevice("joint_left_wheel")
    motor_3 = robot.getDevice("joint_left2_wheel")  
    
    # set position for robot motors
    motor_1.setPosition(float('inf'))
    motor_2.setPosition(float('inf'))
    motor_3.setPosition(float('inf'))
    
    # set velocity for robot motors
    motor_1.setVelocity(0.0)
    motor_2.setVelocity(0.0)
    motor_3.setVelocity(0.0)
    
    # define distance sensors
    sonar_1 = robot.getDevice("sonar_1")
    sonar_2 = robot.getDevice("sonar_2")
    sonar_3 = robot.getDevice("sonar_3")
    
    # enable distance sensors
    sonar_1.enable(timestep)
    sonar_2.enable(timestep)
    sonar_3.enable(timestep)
    
    # define position sensors
    pos_1 = robot.getDevice("joint_right_wheel_sensor")
    pos_2 = robot.getDevice("joint_left_wheel_sensor")
    pos_3 = robot.getDevice("joint_left2_wheel_sensor")  
    
    # enable position sensors
    pos_1.enable(timestep)
    pos_2.enable(timestep)
    pos_3.enable(timestep)
    
    wheel_cirum = 2 * math.pi * WHEEL_RADIUS
    encoder_unit = wheel_cirum / (2*math.pi)
    
    # define and enable lidar sensors
    # lidar = robot.getDevice("head_hokuyo_sensor");
    # lidar.enable(timestep);
    
    # define and enable gps
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    
    # define and enable compass
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    
    
    # define position sensor variables
    # wheel_circum = 2 * math.pi * WHEEL_RADIUS
    # encoder_unit = wheel_circum/(2 * math.pi)
    
    
    # graphic panel defined here
    # display = robot.getDevice('display')
    # px = [10,20,10, 0]
    # py = [0, 10,20,10]
    # display.drawPolygon(px,py)
    
    
    # Measurment_color = 'r'#0x44db09
    # Estimate_color = 'g'#0xd48750
    # Localization_color = 'b'#0x11f0a5
    # Reality_color = 'r'#0xd43343
    # text_width_allignment = int(display.getWidth()*0.05)
    # display.setFont("UbuntuMono-Regular",8,False)
    # display.setColor(Measurment_color)
    # display.drawText("Measurment ⬤",0+text_width_allignment,0)
    # display.setColor(Estimate_color)
    # display.drawText("Estimate ⬤",80+text_width_allignment,0)
    # display.setColor(Localization_color )
    # display.drawText("Localization ⬤",160+text_width_allignment,0)
    # display.setColor(Reality_color )
    # display.drawText("Reality ⬤",260+text_width_allignment,0)
    
    # initiate map grid
    # grid_width = display.getWidth()*0.9
    # grid_height = display.getHeight()*0.9
    # grid_x = display.getWidth()*0.05
    # grid_y = display.getHeight()*0.05
    # section_width = 10#display.getWidth()*0.05
    # section_height = 10#display.getHeight()*0.05
    # display.setColor(0x524f46)
    # for i in range(int(grid_width/section_width)+1):
        # display.drawLine(int(grid_x+i*section_width),int(grid_y),int(grid_x+i*section_width),int(grid_y+grid_height)-5)
    # for i in range(int(grid_height/section_height)+1):
        # display.drawLine(int(grid_x),int(grid_y+i*section_height),int(grid_x+grid_width),int(grid_y+i*section_height))
          
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getMotor('motorname')
    #  ds = robot.getDistanceSensor('dsname')
    #  ds.enable(timestep)
    
    
    # xs = []
    # ys = []
    # ani = FuncAnimation(fig, animate, fargs=(), interval=640)
    

    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        # read GPS values
        gps_values = gps.getValues()
    
        # read compass and rotate arrow accordingly
        compass_val = compass.getValues()
    
        # read sonar sensors values
        sonar_value = np.array([sonar_3.getValue(),sonar_1.getValue(),sonar_2.getValue()])
        
        # read position sensors values
        position_value = encoder_unit*np.array([pos_1.getValue(),pos_2.getValue(),pos_3.getValue()])

    
        # Process sensor data here.   

        
        # updating the current theta
        robot_position[2] = math.atan2(compass_val[0], compass_val[1]);
        
        # updating the currnet robot position
        robot_position[0] = gps_values[0];
        robot_position[1] = gps_values[1];
        
       
        
        # states here
        if (robot_state == START):
        ####################
            # print("START")
            m_line_angle = calc_m_line(robot_position,goal_postition)
            m_line_point_1 = np.array([goal_postition[0],goal_postition[1]])
            m_line_point_2 = np.array([robot_position[0],robot_position[1]])
            # wall following sensor is the main sensor
            wall_following_sensor = 0
            if abs(m_line_angle - robot_position[2]) > math.pi:
                robot_omega = np.array([1.0,1.0,1.0])
            else:    
                robot_omega = np.array([-1.0,-1.0,-1.0])
            robot_state = ROTATE_ROBOT_HEAD_TOWARD_GOAL
        ####################
        elif (robot_state == ROTATE_ROBOT_HEAD_TOWARD_GOAL):
        ####################
            # print("ROTATE_ROBOT_HEAD_TOWARD_GOAL")
            if abs(m_line_angle - robot_position[2]) < angle_threshold:
                robot_omega = np.array([0.0,0.0,0.0])
                robot_state = MOVE_TOWARD_T_ALONG_M_LINE
        ####################
        elif (robot_state == MOVE_TOWARD_T_ALONG_M_LINE):
        ####################
            # print("MOVE_TOWARD_T_ALONG_M_LINE")
            if  finished(step_timer):
                robot_omega = np.array([0.0,0.0,0.0])
                step_timer = step_timer_reset_value
                robot_state = SCAN_ENVIRONMENT
            else:
                robot_omega = forward_sensor_direction[wall_following_sensor]
                step_timer = reduce(step_timer)
        ####################
        elif (robot_state == SCAN_ENVIRONMENT):
        ####################
            # print("SCAN_ENVIRONMENT")
            if scan_timer == store_pose2:
                init_pose2 = robot_position[2]
                scan_timer = reduce(scan_timer)
            elif scan_timer == scan_left_side:
                if abs(init_pose2 - robot_position[2]) < robot_scan_angle :
                    robot_omega = np.array([-1.0,-1.0,-1.0])
                    # read sensors here
                    if is_in_range(sonar_value[wall_following_sensor],hit_threshold_min,hit_threshold_max):
                        scan_timer = scan_timer_reset_value
                        wall_following_sensor = wall_following_sensor                        
                        robot_state = FOLLOW_BOUNDRAY 
                    elif is_in_range(sonar_value[right_sonar(wall_following_sensor)],hit_threshold_min,hit_threshold_max):
                        scan_timer = scan_timer_reset_value
                        wall_following_sensor = right_sonar(wall_following_sensor)
                        robot_state = FOLLOW_BOUNDRAY 
                else:
                    robot_omega = np.array([0.0,0.0,0.0])
                    scan_timer = reduce(scan_timer)
            elif scan_timer == reset_head_dir:
                if abs(init_pose2 - robot_position[2]) > angle_threshold:
                    robot_omega = np.array([1.0,1.0,1.0])
                else:
                    robot_omega = np.array([0.0,0.0,0.0])
                    scan_timer = reduce(scan_timer)
            elif scan_timer == scan_right_side:
                if abs(init_pose2 - robot_position[2]) < robot_scan_angle :
                    robot_omega = np.array([1.0,1.0,1.0])
                    # read sensors here
                    if is_in_range(sonar_value[wall_following_sensor],hit_threshold_min,hit_threshold_max):
                        scan_timer = scan_timer_reset_value
                        wall_following_sensor = wall_following_sensor                        
                        robot_state = FOLLOW_BOUNDRAY 
                    elif is_in_range(sonar_value[right_sonar(wall_following_sensor)],hit_threshold_min,hit_threshold_max):
                        scan_timer = scan_timer_reset_value
                        wall_following_sensor = right_sonar(wall_following_sensor)
                        robot_state = FOLLOW_BOUNDRAY 
                else:
                    robot_omega = np.array([0.0,0.0,0.0])
                    scan_timer = reduce(scan_timer)
            elif finished(scan_timer):
                if abs(init_pose2 - robot_position[2]) > angle_threshold:
                    robot_omega = np.array([-1.0,-1.0,-1.0])
                else:
                    robot_omega = np.array([0.0,0.0,0.0])
                    scan_timer = scan_timer_reset_value
                    robot_state = MOVE_TOWARD_T_ALONG_M_LINE      
        ####################
        elif (robot_state == FOLLOW_BOUNDRAY):
        ####################
            if finished(follow_wall_timer):
                follow_wall_timer = follow_wall_timer_reset_value
                robot_state = EDGE_DETECTION
                scan_timer = scan_timer_reset_value
                circular_move_timer = circular_move_timer_reset_value
                # one_step_estimated_displacement = np.sum(curr_accumulated_estimated_displacement)
                # print("ONE : ",one_step_estimated_displacement)
                # print("left: ",left_move_counter)
                # print("right: ",right_move_counter)
                # print("------------")
                left_move_counter,right_move_counter = 0,0
                # curr_accumulated_estimated_displacement = 0.0
                
            else:
                follow_wall_timer = reduce(follow_wall_timer)
                if (abs(left_move_counter + right_move_counter -follow_wall_timer_reset_value) < bot_stop_threshold):
                    if(circular_move_timer > 3000):
                        follow_wall_timer += 1
                        circular_move_timer = reduce(circular_move_timer)
                        robot_omega = -0.4*forward_sensor_direction[wall_following_sensor]
                    elif(circular_move_timer <= 3000) & (circular_move_timer > 1):
                        follow_wall_timer += 1
                        circular_move_timer = reduce(circular_move_timer)
                        robot_omega = circular_movement_counter_clockwise[wall_following_sensor]
                        # prevend hitting the wall
                        if is_in_range(sonar_value[right_sonar(wall_following_sensor)],hit_threshold_min,hit_threshold_max):
                            robot_omega = np.array([0.0,0.0,0.0])
                            circular_move_timer = 1
                    elif(circular_move_timer == 1):
                        circular_move_timer = circular_move_timer_reset_value
                        wall_following_sensor = right_sonar(wall_following_sensor)
                        left_move_counter,right_move_counter = 0,0
                        follow_wall_timer = follow_wall_timer_reset_value
                        scan_timer = scan_timer_reset_value
                        circular_move_timer = circular_move_timer_reset_value
                        step_timer = step_timer_reset_value
                elif exceeds_upper_limit(sonar_value[wall_following_sensor],hit_threshold_max):
                    robot_omega = 2*rotational_movement_counter_clockwise[wall_following_sensor]
                    left_move_counter += 1
                elif exceeds_lower_limit(sonar_value[wall_following_sensor],hit_threshold_min):
                    robot_omega = 2*rotational_movement_clockwise[wall_following_sensor]
                    right_move_counter += 1
                else:
                    # robot_omega = forward_sensor_direction[wall_following_sensor]
                    robot_omega = 0.25*parallel_move[right_motor(wall_following_sensor)] + 0.25*forward_sensor_direction[wall_following_sensor]
        ####################              
        elif (robot_state == EDGE_DETECTION):
        ####################
            # print("SCAN_ENVIRONMENT")
            if scan_timer == store_pose2:
                init_pose2 = robot_position[2]
                scan_timer = reduce(scan_timer)
            elif scan_timer == scan_left_side:
                if abs(init_pose2 - robot_position[2]) < robot_scan_angle :
                    robot_omega = np.array([-1.0,-1.0,-1.0])
                    # read sensors here
                    if is_in_range(sonar_value[right_sonar(wall_following_sensor)],hit_threshold_min,hit_threshold_max):
                        scan_timer = scan_timer_reset_value
                        wall_following_sensor = right_sonar(wall_following_sensor)
                        robot_state = FOLLOW_BOUNDRAY
                else:
                    robot_omega = np.array([0.0,0.0,0.0])
                    scan_timer = reduce(scan_timer)
            elif scan_timer == reset_head_dir:
                if abs(init_pose2 - robot_position[2]) > angle_threshold:
                    robot_omega = np.array([1.0,1.0,1.0])
                else:
                    robot_omega = np.array([0.0,0.0,0.0])
                    scan_timer = reduce(scan_timer)
            elif scan_timer == scan_right_side:
                if abs(init_pose2 - robot_position[2]) < robot_scan_angle :
                    robot_omega = np.array([1.0,1.0,1.0])
                    # read sensors here
                    if is_in_range(sonar_value[right_sonar(wall_following_sensor)],hit_threshold_min,hit_threshold_max):
                        scan_timer = scan_timer_reset_value
                        wall_following_sensor = right_sonar(wall_following_sensor)
                        robot_state = FOLLOW_BOUNDRAY
                else:
                    robot_omega = np.array([0.0,0.0,0.0])
                    scan_timer = reduce(scan_timer)
            elif finished(scan_timer):
                if abs(init_pose2 - robot_position[2]) > angle_threshold:
                    robot_omega = np.array([-1.0,-1.0,-1.0])
                else:
                    robot_omega = np.array([0.0,0.0,0.0])
                    scan_timer = scan_timer_reset_value
                    robot_state = FOLLOW_BOUNDRAY    
                    follow_wall_timer = follow_wall_timer_reset_value
        ####################                    
        # elif (robot_state == SWAP_FOLLOWING_WALL):
        ####################
            # robot_omega = np.array([0.0,0.0,0.0])
        ####################
        #elif (robot_state == SWAP_ACTIVE_SONAR):

        #elif (robot_state == STOP):

        # elif (robot_state == TEST):
            # robot_omega = [0,0,0]
        
        # robot_omega = [3+0,0+3,-3-3]

        # print(sonar_value)
        # print(wall_following_sensor)
        # print("robo_omega  >> ",robot_omega)
        # print(one_step_estimated_displacement)
        

        
        # update motor velocities
        # add noise to motor velocity
        # noise = np.random.normal(0, 0.15, size=(3, 1))
        motor_1.setVelocity(robot_omega[0])
        motor_2.setVelocity(robot_omega[1])
        motor_3.setVelocity(robot_omega[2])
        
        
        
        # define required coeffecients
        # initial_x = 120
        # initial_y = -120
        # x_bias = initial_x + grid_x
        # y_bias = initial_y + grid_y
        # x_expansion = 5
        # y_expansion = 5
        
        # estimate robot movement
        # estimate_displacement = WHEEL_RADIUS * kinematic(robot_position[2], robot_omega)
        
        
        # update the estimation position
        # display.setColor(Estimate_color)
        # display.drawPixel(int(robot_pos_estimate[0]*x_expansion+x_bias),int(-robot_pos_estimate[1]*y_expansion+y_bias))
        # draw_point_on_plot(estimate_displacement[0],estimate_displacement[1],Estimate_color)
        
        
        # measure robot movement
        # robot_phi_measure = position_value - robot_phi_measure
        # measure_phi = robot_phi_measure / WHEEL_RADIUS
        # estimate_displacement = WHEEL_RADIUS*kinematic(robot_position[2], measure_phi)
        # robot_pos_measure[0] += estimate_displacement[0]
        # robot_pos_measure[1] += estimate_displacement[1]
        
        
        # update robot measurement 
        # display.setColor(Measurment_color)
        # display.drawPixel(int(robot_pos_measure[0]*x_expansion+x_bias),int(-robot_pos_measure[1]*y_expansion+y_bias))
        # draw_point_on_plot(robot_pos_measure[0],robot_pos_measure[1],Measurment_color)
        
        # updating the real position
        # display.setColor(Reality_color)
        # display.drawPixel(int(gps_values[0]*x_expansion+x_bias),int(-gps_values[1]*y_expansion+y_bias))
        # draw_point_on_plot(gps_values[0],gps_values[2],Reality_color)
        
        # print(robot_phi_measure)
        # print(pos_1.getType()) is rotational? yep.
        # print("---------")
        
        
        
        # if(len(data)>20):
            # ax.clear()
            # for point in data:
                # ax.scatter(point[0],point[1],color=point[2])
            # figure.canvas.draw()
            # figure.canvas.flush_events()
            # data.clear()
        # data.append([[estimate_displacement[0],estimate_displacement[1],Estimate_color]
                           # ,[robot_pos_measure[0],robot_pos_measure[1],Measurment_color]
                           # ,[gps_values[0],gps_values[2],Reality_color]])
        
        
    pass
    
    # Enter here exit cleanup code.
    



    