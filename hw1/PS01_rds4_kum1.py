################################################################################
##                      Student code - hand this section in                   ##
################################################################################

# Ryan Spring - rds4
# Kori Macdonald - kum1
# 2/5/2015

########  part 0: helper functions ########
# global PWM setting for all motion
# you might need to increase this a bit if your robot doesn't move
MOTOR_PWM = 65

# stops the robot for the argument time
# arguments: time
# return: nothing
# Kori Macdonald - kum1
def move_stop(time):
    # student code start
    rone.motor_brake('l')
    rone.motor_brake('r')
    sys.sleep(time) 
    # student code start
    

# moves forward for the argument time
# arguments: time
# return: nothing
# Kori Macdonald - kum1
def move_forward(time):
    # student code start
    rone.motor_set_pwm('l',MOTOR_PWM)
    rone.motor_set_pwm('r',MOTOR_PWM)
    sys.sleep(time)
    rone.motor_brake('l')
    rone.motor_brake('r')
    # student code end

# moves backward for the argument time
# arguments: time
# return: nothing
# Kori Macdonald - kum1
def move_backward(time):
    # student code start
    rone.motor_set_pwm('l',-MOTOR_PWM)
    rone.motor_set_pwm('r',-MOTOR_PWM)
    sys.sleep(time)
    rone.motor_brake('l')
    rone.motor_brake('r')
    # student code end

    
# rotate right for the argument time
# arguments: time
# return: nothing
# Kori Macdonald - kum1
def move_rotate_right(time):
    # student code start
    rone.motor_set_pwm('l',MOTOR_PWM)
    rone.motor_brake('r')
    sys.sleep(time)
    rone.motor_brake('l')
    # student code end
    

# rotate left for the argument time
# arguments: time
# return: nothing
# Kori Macdonald - kum1
def move_rotate_left(time):
    # student code start
    rone.motor_set_pwm('r',MOTOR_PWM)
    rone.motor_brake('l')
    sys.sleep(time)
    rone.motor_brake('r')
    # student code end
    
# backwards rotate right for the argument time
# arguments: time
# return: nothing
# Ryan Spring - rds4 and Kori Macdonald - kum1
def back_rotate_right(time):
    # student code start
    rone.motor_set_pwm('l',-MOTOR_PWM)
    rone.motor_brake('r')
    sys.sleep(time)
    rone.motor_brake('l')
    # student code end
    

# backwards rotate left for the argument time
# arguments: time
# return: nothing
# Ryan Spring - rds4 and Kori Macdonald - kum1
def back_rotate_left(time):
    # student code start
    rone.motor_set_pwm('r',-MOTOR_PWM)
    rone.motor_brake('l')
    sys.sleep(time)
    rone.motor_brake('r')
    # student code end
    

########  part 1: square motion ########

# drives the robot in a square
# arguments: nothing
# return: nothing
# Ryan Spring - rds4 and Kori Macdonald - kum1
def square_motion():
    # wait for 2 seconds to let you unplug the robot...
    sys.sleep(2000)
    for i in range(1,5):
        # student code start
        move_forward(1500)
        move_rotate_right(650)
        # student code end
    move_stop(100)


########  part 2: move towards light ########
# compute and return the difference between the left and the right light sensor
# arguments: nothing
# return: difference between left sensor and right sensor
# Ryan Spring - rds4 and Kori Macdonald - kum1
def light_diff():
    # student code start
    light_fl = rone.light_sensor_get_value('fl')
    light_fr = rone.light_sensor_get_value('fr')
    diff = light_fl - light_fr
    # student code end
    return diff


# Move towards Light! Use the structure below, and the movement helper functions from above
# arguments: nothing
# return: nothing
# Ryan Spring - rds4 and Kori Macdonald - kum1
def light_follow():
    diff_start = light_diff()
    print "diff_start", diff_start
    sys.sleep(1000)
    while True:
        diff = light_diff() - diff_start
        print "diff", diff
        
        # student code start
        if light_diff() > 0:
            move_rotate_left(10)
        else:
            move_rotate_right(10)
        move_forward(10)
        #diff_start = light_diff()
        # student code end
        
########  part 3: avoid obstacles with bump sensors ########
# Checks the bump sensor for impacts from the left
# arguments: nothing
# return: True if the bump sensor is pressed from the left, False otherwise
# Ryan Spring - rds4
def bump_left_get_value():
    left = [0, 1, 2]
    bump_bits = rone.bump_sensors_get()
    for n in left:
        if n in bump_bits:
            return True
    return False
    #return ((bump_bits & 7) > 0)

# Checks the bump sensor for impacts from the front
# arguments: nothing
# return: True if the bump sensor is pressed from the front, False otherwise
# Ryan Spring - rds4
def bump_front_get_value():
    front = [0, 7]
    bump_bits = rone.bump_sensors_get()
    for n in front:
        if n in bump_bits:
            return True
    return False    #return (bump_bits == 129)

# Checks the bump sensor for impacts from the right
# arguments: nothing
# return: True if the bump sensor is pressed from the right, False otherwise
# Ryan Spring - rds4
def bump_right_get_value():
    right = [7, 6, 5]
    bump_bits = rone.bump_sensors_get()
    for n in right:
        if n in bump_bits:
            return True
    return False    #return ((bump_bits & 224) > 0)

# Move the robot away from obstacles using the bump sensors
# arguments: nothing
# return: nothing
# Ryan Spring - rds4 and Kori Macdonald - kum1
def bump_avoid():
    left = True
    print "bump_avoid()"
    sys.sleep(1000)
    while True:
        # student code start
        if bump_left_get_value():
            left = True      
            back_rotate_right(1000)
        elif bump_right_get_value():
            left = False
            back_rotate_left(1000)
        elif bump_front_get_value():                  
            if left:
                back_rotate_right(1000)
            else:
                back_rotate_left(1000)
        else:
            move_forward(500)
                
        # student code end
        
########  part 4: avoid obstacles with IR sensors ########

# Use the IR sensors to detect obstacles
# arguments: nothing
# return: tuple of booleans (obs_front, obs_left, obs_right)
def obstacle_detect():
    obs_front = False
    obs_left = False
    obs_right = False
    rone.ir_comms_send_message();
    sys.sleep(20)
    msg = rone.ir_comms_get_message()
    if msg != None:
        (msg, recv_list, xmit_list, range_bits) = msg
        if (0 in recv_list) and (7 in recv_list):
            obs_front = True
        if (0 in recv_list) or (1 in recv_list):
            obs_left = True
        if (6 in recv_list) or (7 in recv_list):
            obs_right = True
    return (obs_front, obs_left, obs_right)


# Move away from obstacles! Use the structure below, and the movement helper functiosn from above
# arguments: nothing
# return: nothing
# Ryan Spring - rds4 and Kori Macdonald - kum1
def obstacle_avoid():
    sys.sleep(1000)
    while True:
        (obs_front, obs_left, obs_right) = obstacle_detect()
        print obs_front, obs_left, obs_right
        # student code start
        if obs_left:
            print "left"
            left = True
            move_rotate_right(100)
        elif obs_right:
            print "right"
            left = False
            move_rotate_left(100)
        elif obs_front:
            print "face"
            if left:
                move_rotate_right(500)
            else:
                move_rotate_left(500)
        else:
            move_forward(500)
        # student code end

#### Pose estimator ####
WHEEL_BASE = 78
ENCODER_MM_PER_TICKS = 0.0625

#update the pose state
# Ryan Spring - rds4 and Kori Macdonald - kum1
def pose_update(pose_state):
    # 1. Get the left and right encoder ticks
    left = rone.encoder_get_ticks("l")
    right = rone.encoder_get_ticks("r")
    
    # 2. Compute the left and right delta ticks
    # Don't forget to use encoder_delta_ticks() to properly compute the change in tick values
    dleft = velocity.encoder_delta_ticks(left, pose_state['ticksL'])
    dright = velocity.encoder_delta_ticks(right, pose_state['ticksR'])

    # 3. compute the left and right distance for each wheel
    # cast the delta ticks from step 2 to floats before you do the distance computation
    dl = float(dleft) * ENCODER_MM_PER_TICKS
    dr = float(dright) * ENCODER_MM_PER_TICKS

    # 4. save the left and right ticks to pose_state so we can measure the difference next time
    pose_state['ticksL'] = left
    pose_state['ticksR'] = right
    
    # 5. Compute the distance traveled by the center of the robot in millimeters
    center = (dr + dl) / 2.0

    # 6. Add the distance to the odometer variable in pose_state
    pose_state['odometer'] = pose_state['odometer'] + abs(center)
    
    # 7. compute the arc angle in radians
    # don't call atan here, use the small angle approximation: arctan(theta) ~ theta
    dtheta = (dr - dl) / float(WHEEL_BASE)
    
    # 8. finally, update x, y, and theta, and save them to the pose state
    # use math2.normalize_angle() to normalize theta before storing it in the pose_state
    l = ((dr - dl) / 2.0) * math.sin(90 - dtheta)
    ntheta = pose_state['theta'] + dtheta
    pose_state['x'] = (center + l) * math.cos(ntheta) + pose_state['x']
    pose_state['y'] = (center + l) * math.sin(ntheta) + pose_state['y']
    pose_state['theta'] = math2.normalize_angle(ntheta)
    return 0

#### Waypoint controller constants ####
MOTION_CAPTURE_DISTANCE = 16
MOTION_RELEASE_DISTANCE = 32
MOTION_CAPTURE_ANGLE = math.pi/2
MOTION_RELEASE_ANGLE = math.pi/10
MOTION_TV_MIN = 20
MOTION_TV_GAIN = 3
MOTION_RV_GAIN = 1300
MOTION_RV_MAX = 7000

# Convert rectangular to polar
# return a tuple of the form (r, theta)
# theta lies between (-pi, pi] 
# Ryan Spring - rds4 and Kori Macdonald - kum1
def topolar(x, y):
    # student code start
    r2 = (x*x) + (y*y)
    r = math.sqrt(r2)
    theta = math.atan2(y,x)
    #student code end
    return (r, theta)

# compute the distance and heading to the goal position
# return a tuple of the form: (goal_distance, goal_heading, robot_heading)
# Ryan Spring - rds4 and Kori Macdonald - kum1
def compute_goal_distance_and_heading(goal_position, robot_pose):
    # student code start
    (goal_distance, goal_heading) = topolar(goal_position[0] - robot_pose[0], goal_position[1] - robot_pose[1])
    robot_heading = robot_pose[2]
    # student code end
    return (goal_distance, goal_heading, robot_heading)

# Compute the smallest angle difference between two angles
# This difference will lie between (-pi, pi]
# Ryan Spring - rds4
def smallest_angle_diff(current_angle, goal_angle):
    # student code start
    if (current_angle >= 0 and goal_angle >= 0) or (current_angle < 0 and goal_angle < 0):
        return math2.normalize_angle(goal_angle - current_angle)
    elif (current_angle >= 0):
        return -math2.normalize_angle(abs(current_angle) + abs(goal_angle))
    else:
        return math2.normalize_angle(abs(current_angle) + abs(goal_angle))
    # student code end

# compute the tv profile for the velocity controller.
# this should match the plot from the handout
# Ryan Spring - rds4 and Kori Macdonald - kum1
def motion_controller_tv(d, tv_max):
    # student code start
    tv_temp = MOTION_TV_GAIN * d + MOTION_TV_MIN
    # student code end
    return math2.bound(tv_temp, tv_max)

# compute the rv controller for the velocity controller
# this should bound the value to MOTION_RV_MAX
# Ryan Spring - rds4 and Kori Macdonald - kum1
def motion_controller_rv(heading, heading_to_goal):
    # student code start
    bearing_error = smallest_angle_diff(heading, heading_to_goal)
    rv = math2.bound(MOTION_RV_GAIN * bearing_error, MOTION_RV_MAX)
    print heading
    print heading_to_goal
    print bearing_error
    print rv
    print "\n"
    # student code end
    return (rv, bearing_error)