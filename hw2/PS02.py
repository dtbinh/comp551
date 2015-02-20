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










        # student code end
        sys.sleep(REMOTE_XMIT_DELAY * 2)


def test_ir_receive():
    # run forever. print a IR message if you get one
    # turn on blue lights if you get a message, red lights if not
    # use the rone.led_set_group() function to turn on lights

    while True:
        # student code start










        # student code end
        sys.sleep(NEIGHBOR_PERIOD * 2)

        

def check_buttons():
    # return a string of the buttons that are pressed 
    # return a blank string if no button is pressed
    buttons = '' # placeholder code
    # student code start






    # student code end
    return buttons


def leader_motion_controller(radio_msg):
    # args: radio_msg, a string of the radio message from the remote robot
    # returns a tuple of (tv, rv) to make te leader drive around
    tv = 0 # placeholder code
    rv = 0 # placeholder code
    print 'radio msg:', radio_msg  # debugging code - comment out when this is working

    # student code start














    # student code end
    return (tv, rv)


def compute_bearing(receivers_list):
    # args: receivers_list, a list of the receivers that this message was received on
    # returns the bearing of the neighbor
    bearing = 0 # placeholder code

    # student code start








    # student code end
    print 'recv',receivers_list, bearing # debugging code - comment out when this is working
    return bearing


def compute_orientation(transmitters_list):
    # args: transmitters_list, a list of the transmitters that this message was sent from
    # returns the orientation of the neighbor
    orientation = 0 # placeholder code


    # student code start









    # student code end

    print 'ornt',transmitters_list, orientation # debugging code - comment out when this is working
    return orientation


def follow_motion_controller(nbr):
    # args: nbr a neighbor to follow
    # returns a tuple of (tv, rv) to make the follower follow
    # use neighborsX.get_nbr_bearing(nbr) to get the bearing of nbr
    # (optional) you can use neighborsX.get_nbr_range_bits(nbr) to stop the 
    # follower when it is close to the leader.  if there are more than 3 range 
    # bits, you should stop
    tv = 0 # placeholder code
    rv = 0 # placeholder code












    # student code end


    return (tv, rv)


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
        if 'r' in buttons:
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





#test_radio_receive()
#test_ir_receive()
#follow_the_leader()