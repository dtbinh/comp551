import sys, rone

########  part 0: helper functions ########
# global PWM setting for all motion
# you might need to increase this a bit if your robot doesn't move
MOTOR_PWM = 65

# stops the robot for the argument time
# arguments: time
# return: nothing
def move_stop(time):
    # student code start
    rone.motor_brake('l')
    rone.motor_brake('r')
    sys.sleep(time) 
    # student code start
    

# moves forward for the argument time
# arguments: time
# return: nothing
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