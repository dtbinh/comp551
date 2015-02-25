import rone, sys, math, velocity, leds, neighborsX

FTL_TV = 50
FTL_RV = math.pi*1000/2
LED_BRIGHTNESS = 40
REMOTE_XMIT_DELAY = 50
NEIGHBOR_PERIOD = 300
IR_XMIT_PERIOD_FUDGE = 30
NEIGHBOR_TIMEOUT = 3 * NEIGHBOR_PERIOD + IR_XMIT_PERIOD_FUDGE

MODE_IDLE = 0
MODE_REMOTE = 1
MODE_LEADER = 2
MODE_FOLLOWER = 3
MODE_SORTED = 4
MODE_FLOCK = 5
MODE_ALIGN = 6

def radio_get_message():
    radio_msg = rone.radio_get_message_usr_newest()
    rone.radio_flush_usr_queue()
    return radio_msg


def test_radio_receive():
    # run forever. print a radio message if you get one
    # turn on blue lights if you get a message, red lights if not
    # use the rone.led_set_group() function to turn on lights
    # use the radio_get_message() helper function above to receive a radio message
    while True:
        # student code start
        msg = radio_get_message()
        sys.sleep(20)
        if msg == None:
            leds._led_set_group('r', 100)
            leds._led_set_group('b', 0)
        else:
            print msg
            leds._led_set_group('r', 0)
            leds._led_set_group('b', 100)
        # student code end
        sys.sleep(REMOTE_XMIT_DELAY * 2)


def test_ir_receive():
    # run forever. print a IR message if you get one
    # turn on blue lights if you get a message, red lights if not
    # use the rone.led_set_group() function to turn on lights
    while True:
        # student code start
        msg = rone.ir_comms_get_message()
        sys.sleep(20)
        if msg == None:
            leds._led_set_group('r', 100)
            leds._led_set_group('b', 0)
        else:
            (msg, recv_list, xmit_list, range_bits) = msg
            print msg
            sys.sleep(20)
            leds._led_set_group('b', 100)
            leds._led_set_group('r', 0)
        # student code end
        sys.sleep(NEIGHBOR_PERIOD * 2)

        

def check_buttons():
    # return a string of the buttons that are pressed 
    # return a blank string if no button is pressed
    buttons = '' # placeholder code
    # student code start
    red = rone.button_get_value('r')
    blue = rone.button_get_value('b')
    green = rone.button_get_value('g')
    
    if red:
        buttons += 'r'
    if blue:
        buttons += 'b'
    if green:
        buttons += 'g'
    # student code end
    return buttons


def leader_motion_controller(radio_msg):
    # args: radio_msg, a string of the radio message from the remote robot
    # returns a tuple of (tv, rv) to make the leader drive around
    tv = FTL_TV
    rv = 0
    print 'radio msg:', radio_msg  # debugging code - comment out when this is working

    # student code start
    if 'r' in radio_msg:
        #left
        rv = FTL_RV
    elif 'g' in radio_msg:
        #forward
        rv = 0
    elif 'b' in radio_msg:
        #right
        rv = -FTL_RV
        
    # student code end
    return (tv, rv)

eighth_pi = 1.0 / 8 * math.pi
receiver_angle = [1, 3, 5, 7, 9, 11, 13, 15]

fourth_pi = 1.0 / 4 * math.pi
transmitter_angle = [0, 1, 2, 3, 4, 5, 6, 7]

def compute_bearing(receivers_list):
    # args: receivers_list, a list of the receivers that this message was received on
    # returns the bearing of the neighbor
    bearing = 0 # placeholder code

    # student code start
    count = 0
    sum = 0
    print receivers_list
    for v in receivers_list:
        count += 1
        sum += receiver_angle[v] * eighth_pi
    
    if (0 in receivers_list) and (7 in receivers_list):
        sum = 0 
        
    if count > 0:
        bearing = sum / count  
    
    # student code end
    print 'recv',receivers_list, bearing # debugging code - comment out when this is working
    return bearing


def compute_orientation(transmitters_list):
    # args: transmitters_list, a list of the transmitters that this message was sent from
    # returns the orientation of the neighbor
    orientation = 0 # placeholder code

    # student code start
    count = 0
    sum = 0
    for v in transmitters_list:
        count += 1
        sum += transmitter_angle[v] * fourth_pi
    
    if (0 in transmitters_list) and (7 in transmitters_list):
        sum = 0 
    
    if count > 0:
        orientation = sum / count
        
    # student code end
    print 'ornt',transmitters_list, orientation # debugging code - comment out when this is working
    return orientation

# Bound theta to lie between (-pi,pi]
def normalize_angle(theta):
    while theta > math.pi:
        theta -= 2 * math.pi
    while theta <= -math.pi:
        theta += 2 * math.pi
    return theta

# Bound value to lie between -value_max and value_max
def bound(value, value_max):
    ## Clamp value between +/- value_max
    if value > value_max:
        value = value_max
    elif value < -value_max:
        value = -value_max
    return value

MOTION_RV_GAIN = 650
MOTION_RV_MAX = 2000

def follow_motion_controller(nbr):
    # args: nbr a neighbor to follow
    # returns a tuple of (tv, rv) to make the follower follow
    # use neighborsX.get_nbr_bearing(nbr) to get the bearing of nbr
    # (optional) you can use neighborsX.get_nbr_range_bits(nbr) to stop the 
    # follower when it is close to the leader.  if there are more than 300 range 
    # bits, you should stop
    tv = 0
    rv = 0

    bearing = neighborsX.get_nbr_bearing(nbr)
    range_bits = neighborsX.get_nbr_range_bits(nbr)
    print "range", range_bits
    sys.sleep(20)
    if range_bits < 300:
        tv = 0
    else:
        tv = FTL_TV
        
    rv = bound(MOTION_RV_GAIN * normalize_angle(bearing), MOTION_RV_MAX)
    # student code end
    return (tv, rv)

def FTL_sorted():
    tv = 0
    rv = 0
    nbr_list = neighborsX.get_neighbors()
    if len(nbr_list) > 0:
        # You have neighbors. Follow max-min robot-id. If you're the lowest, become leader.
        follow_id = -1
        follow_nbr = None
        for nbr in nbr_list:
            nbr_id = neighborsX.get_nbr_id(nbr)
            if nbr_id < rone.get_id() and nbr_id > follow_id:
                follow_id = nbr_id
                follow_nbr = nbr
        
        if follow_id == -1: # leader
            leds.set_pattern('rg','blink_fast', LED_BRIGHTNESS)
            tv =  FTL_TV
            rv = 0
            #bump_avoid()
        else: # follower
            leds.set_pattern('b','blink_fast', LED_BRIGHTNESS)
            (tv, rv) = follow_motion_controller(follow_nbr)
    else:       
        # no neighbors. do nothing
        leds.set_pattern('r','circle',LED_BRIGHTNESS)
    velocity.set_tvrv(tv, rv)
    
def neighbor_heading(nbr):
    # finding neighbor heading
    orientation = neighborsX.get_nbr_orientation(nbr)
    bearing = neighborsX.get_nbr_bearing(nbr)
    heading = bearing + math.pi - orientation
    return normalize_angle(heading)
    
def match_orientation():
    # pair-wise orientation match from high-id to low-id
    tv = 0
    rv = 0

    nbr_list = neighborsX.get_neighbors()
    if len(nbr_list) > 0:
        # You have neighbors. Follow any neighbor.  get the first neighbor
        leds.set_pattern('rb','blink_fast', LED_BRIGHTNESS)
        nbr = nbr_list[0]
        rv = bound(MOTION_RV_GAIN * neighbor_heading(nbr), MOTION_RV_MAX)
    else:
        #no neighbors. do nothing
        leds.set_pattern('b','circle',LED_BRIGHTNESS)
    # student code end
    velocity.set_tvrv(tv, rv)

def average_orientation():
    # calculate average heading of all neighbors
    # compute vector sum of orientation angles and then average
    nbr_list = neighborsX.get_neighbors()
    if len(nbr_list) > 0:
        # You have neighbors
        vector_sum = [0, 0]
        for nbr in nbr_list:
            nbr_heading = neighbor_heading(nbr)
            vector_sum[0] += math.cos(nbr_heading)  
            vector_sum[1] += math.sin(nbr_heading)
        average_heading = math.atan2(vector_sum[1], vector_sum[0])
        return (FTL_TV, bound(MOTION_RV_GAIN * average_heading, MOTION_RV_MAX))
    else:
        leds.set_pattern('r','circle',LED_BRIGHTNESS)
    return (0, 0)

def flock():
    # have the robots follow the leader as a flock
    global FTL_leader_tv, FTL_leader_rv

    tv = 0
    rv = 0
    nbr_list = neighborsX.get_neighbors()
    if len(nbr_list) > 0:
        leds.set_pattern('bg','blink_fast', LED_BRIGHTNESS)
        (tv, rv) = average_orientation()
        
        radio_msg = radio_get_message()
        if radio_msg != None:
            # we have a message! put lights in active mode
            (FTL_leader_tv, FTL_leader_rv) = leader_motion_controller(radio_msg)
            rv += FTL_leader_rv
    else:       
        # no neighbors. do nothing
        leds.set_pattern('r','circle',LED_BRIGHTNESS)
    velocity.set_tvrv(tv, rv)
    

################################################################################
##                     PS07 Distribution code                                 ## 
##                 Please read.  Do not need to edit or hand in               ## 
################################################################################
########  part 0: helper functions ########
# global PWM setting for all motion
# you might need to increase this a bit if your robot doesn't move
MOTOR_PWM = 65

def move_forward(time):
    # student code start
    rone.motor_set_pwm('l',MOTOR_PWM)
    rone.motor_set_pwm('r',MOTOR_PWM)
    sys.sleep(time)
    rone.motor_brake('l')
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
    return False

# Move the robot away from obstacles using the bump sensors
# arguments: nothing
# return: nothing
# Ryan Spring - rds4 and Kori Macdonald - kum1
def bump_avoid():
    left = True
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

def FTL_remote():
    buttons = check_buttons()

    if buttons != '':
        leds.set_pattern('r','blink_fast',LED_BRIGHTNESS)
        rone.radio_send_message(buttons)
    else:
        leds.set_pattern('r','circle',LED_BRIGHTNESS)
    #sleep for a bit to avoid continuous radio transmission
    sys.sleep(REMOTE_XMIT_DELAY)


# These variables are used as static variables for FTL_leader
radio_msg_time = sys.time()
FTL_leader_tv = 0
FTL_leader_rv = 0

def FTL_leader():
    global radio_msg_time, FTL_leader_tv, FTL_leader_rv
    radio_msg = radio_get_message()

    if radio_msg != None:
        # we have a message! put lights in active mode
        leds.set_pattern('g','blink_fast',LED_BRIGHTNESS)
        radio_msg_time = sys.time()
        (FTL_leader_tv, FTL_leader_rv) = leader_motion_controller(radio_msg)
    else:
        # no message. check for radio message timeout
        if sys.time() > (radio_msg_time + REMOTE_XMIT_DELAY * 3):
            # message timeout.  stop the motors
            FTL_leader_tv = 0
            FTL_leader_rv = 0
        leds.set_pattern('g','circle',LED_BRIGHTNESS)
    velocity.set_tvrv(FTL_leader_tv, FTL_leader_rv)


def FTL_follower():
    # act on the information from the nbr state.  Note that this might be 
    # information stored from the last message we received, because message 
    # information remains active for a while
    
    tv = 0
    rv = 0
    nbr_list = neighborsX.get_neighbors()
    if len(nbr_list) > 0:
        # You have neighbors. Follow any neighbor.  get the first neighbor
        leds.set_pattern('b','blink_fast', LED_BRIGHTNESS)
        nbr = nbr_list[0]
        (tv, rv) = follow_motion_controller(nbr)
    else:
        #no neighbors. do nothing
        leds.set_pattern('b','circle',LED_BRIGHTNESS)
    velocity.set_tvrv(tv, rv)


def leds_blink_all(brightness):
    if (sys.time() % 500) < 250:
        rone.led_set_group('r', brightness)
        rone.led_set_group('g', brightness)
        rone.led_set_group('b', brightness)
    else:
        rone.led_set_group('r', 0)
        rone.led_set_group('g', 0)
        rone.led_set_group('b', 0)


def follow_the_leader():
    mode = MODE_IDLE

    velocity.init(0.22, 40, 0.3, 0.1)
    leds.init()
    neighborsX.init(NEIGHBOR_PERIOD, compute_bearing, compute_orientation)
    
    while mode == MODE_IDLE:
        # First, wait for the user to press a button, then select states
        # The user is already pressing a button.  wait until they release
        while check_buttons() != '':
            leds_blink_all(LED_BRIGHTNESS)
        # wait for a button press
        while check_buttons() == '':
            leds_blink_all(LED_BRIGHTNESS)
            
        # finally, process the button information
        buttons = check_buttons()
        print buttons
        if 'rg' == buttons:
            mode = MODE_SORTED
        elif 'bg' == buttons:
            mode = MODE_FLOCK
        elif 'rb' == buttons:
            mode = MODE_ALIGN
        elif 'r' in buttons:
            mode = MODE_REMOTE
        elif 'g' in buttons:
            mode = MODE_LEADER
        elif 'b' in buttons:
            mode = MODE_FOLLOWER

    # Now that you know your mode, run the main loop
    while True:
        # run the neighbor system
        neighborsX.update()

        # run the velocity controller
        velocity.update()
        
        # update the led animations
        leds.update()

        if mode == MODE_REMOTE:                
            FTL_remote()
        elif mode == MODE_LEADER:
            FTL_leader()
        elif mode == MODE_FOLLOWER:
            FTL_follower()
        elif mode == MODE_SORTED:
            FTL_sorted()
        elif mode == MODE_FLOCK:
            flock()
        elif mode == MODE_ALIGN:
            match_orientation()


#test_radio_receive()
#test_ir_receive()
follow_the_leader()
