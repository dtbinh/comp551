import rone, sys, math, velocity, leds, neighborsX

FTL_TV = 100
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
        if radio_get_message() == None:
            rone.led_set_group('r', 100)
        else:
            rone.led_set_group('b', 100)
        # student code end
        sys.sleep(REMOTE_XMIT_DELAY * 2)


def test_ir_receive():
    # run forever. print a IR message if you get one
    # turn on blue lights if you get a message, red lights if not
    # use the rone.led_set_group() function to turn on lights
    while True:
        # student code start
        rone.ir_comms_send_message();
        sys.sleep(20)
        msg = rone.ir_comms_get_message()
        if msg == None:
            rone.led_set_group('r', 100)
        else:
            (msg, recv_list, xmit_list, range_bits) = msg
            print msg
            sys.sleep(20)
            rone.led_set_group('b', 100)
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

eighth_pi = math.pi / 8
receiver_angle = [1, 3, 5, 7, 9, 11, 13, 15]

fourth_pi = math.pi / 4
transmitter_angle = [0, 1, 2, 3, 4, 5, 6, 7]

def compute_bearing(receivers_list):
    # args: receivers_list, a list of the receivers that this message was received on
    # returns the bearing of the neighbor
    bearing = 0 # placeholder code

    # student code start
    count = 0
    sum = 0
    for v in receivers_list:
        count += 1
        sum += receiver_angle[v] * eighth_pi
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

MOTION_RV_GAIN = 1300
MOTION_RV_MAX = 7000

def follow_motion_controller(nbr):
    # args: nbr a neighbor to follow
    # returns a tuple of (tv, rv) to make the follower follow
    # use neighborsX.get_nbr_bearing(nbr) to get the bearing of nbr
    # (optional) you can use neighborsX.get_nbr_range_bits(nbr) to stop the 
    # follower when it is close to the leader.  if there are more than 3 range 
    # bits, you should stop
    tv = 0
    rv = 0

    bearing = neighborsX.get_nbr_bearing(nbr)
    range_bits = neighborsX.get_nbr_range_bits(nbr)

    if range_bits > 3:
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
        # You have neighbors. Follow max-min robot-id. if you're the lowest, become leader.
        follow_id = -1
        follow_nbr = None
        for nbr in nbr_list:
            nbr_id = neighborsX.get_nbr_id(nbr)
            if nbr_id < rone.get_id() and nbr_id > follow_id:
                follow_id = nbr_id
                follow_nbr = nbr
        
        if follow_id == -1: # leader
            leds_blink_all(LED_BRIGHTNESS)
            tv =  FTL_TV
            rv = FTL_RV
        else: # follower
            leds.set_pattern('b','blink_fast', LED_BRIGHTNESS)
            (tv, rv) = follow_motion_controller(follow_nbr)
    else:       
        # no neighbors. do nothing
        leds.set_pattern('r','circle',LED_BRIGHTNESS)
    velocity.set_tvrv(tv, rv)
    
def match_orientation():
    # TODO - pair-wise orientation match from high-id to low-id
    return

def average_orientation():
    # TODO - calculate average heading of all neighbors
    # compute vector sum of orientation angles and then average
    return

def flock():
    # TODO - have the robots follow the leader as a flock
    return
    

################################################################################
##                     PS07 Distribution code                                 ## 
##                 Please read.  Do not need to edit or hand in               ## 
################################################################################


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
        if 'rg' == buttons:
            mode = MODE_SORTED
        elif 'gb' == buttons:
            mode = MODE_FLOCK
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





#test_radio_receive()
#test_ir_receive()
#follow_the_leader()
