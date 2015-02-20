################################################################################
##                         Neighbor System Module                             ##
################################################################################
##    NAME
##        Neighbor
##
##    DESCRIPTION
##        Neighbor System Module
##
##    FUNCTIONS
##        init(nbr_period, orientation_enable)
##            initialize the nbr_state dicitionary
##
##            PARAMS
##                1. nbr_period         - (?)
##                2. orientation_enable - boolean for the orientation (?)
##                
##            RETURN
##                the nbr_state dictionary
##                
##        _process_nbr_message(msg, receivers_list, current_time)
##            processing the received message from neighboring robots
##
##            PARAMS
##                1. msg           - the message received
##                2. receiver_list - the list of receivers the robot has in order to commute the bearing of where the message was received
##                3. current_time  - the current time
##
##        set_message(message)
##            sets the nbr_state dictionary's 'message' key with a new value
##
##            PARAMS
##                1. message   - the new message to be stored into the nbr_state dictionary
##                
##        get_message(message)
##            gets the nbr_state dictionary's 'message' key
##
##            PARAMS
##                1. message   - (?) why this is needed?
##                
##            RETURN
##                the String message stored in nbr_state
##
##        get_neighbors()
##            gets the nbr_state dictionary's 'nbr_list' key
##                
##            RETURN
##                the nbr_list stored in nbr_state
##                (each neighbor in nbr_list is the tuple: (nbr_id, message, bearing, orientation, orientation_active, range_bits, msg_time))
##
##
##        get_nbr_id(nbr)
##            gets the neighbor id for the input nbr
##
##            PARAMS
##                1. nbr - the neighbor for getting the id
##                
##            RETURN
##                the integer value of the nbr id
##
##        get_nbr_message(nbr)
##            gets the message for the input nbr
##
##            PARAMS
##                1. nbr - the neighbor for getting the message
##
##            RETURN
##                the String of the nbr's message
##
##        get_nbr_bearing(nbr)
##            gets the neighbor bearing for the input nbr
##
##            PARAMS
##                1. nbr - the neighbor for getting the bearing
##
##            RETURN
##                the integer value of the nbr bearing
##
##        
##        get_nbr_orientation(nbr)
##            gets the neighgets the neighbor orientation for the input nbrbor orientation for the input nbr
##
##            PARAMS
##                1. nbr - the neighbor for getting the orientation
##
##            RETURN
##                the value of the nbr orientation
##
##        get_nbr_orientation_active(nbr)
##            gets the neighbor orientation active for the input nbr
##
##            PARAMS
##                1. nbr - the neighbor for getting the orientation active
##
##            RETURN
##                the nbr orientation active
##
##        get_nbr_range_bits(nbr)
##            gets the neighbor range bits for the input nbr. more bits means closer neighbors
##
##            PARAMS
##                1. nbr - the neighbor for getting the close range
##
##            RETURN
##                the integer value of the nbr close range
##
##        get_nbr_msg_time(nbr)
##            gets the neighbor message time for the input nbr
##
##            PARAMS
##                1. nbr - the neighbor for getting the message time
##
##            RETURN
##                the integer value of the nbr message time
##
##        update()
##            updates the state of the nbr_state by checking the IR for new messages
##
##        _queue_ir_message(ir_queue, msg)
##            queue is IR_QUEUE_XMIT_SIZE deep. throw away extra messages if queue full
##
##            PARAMS
##                1. ir_queue - a list of the IR messages(?)
##                2. msg      - the new message to enter the IR queue
##
##        _tochar(val)
##            converting integers to string
##
##            PARAMS
##                1. val - an integer value
##
##            RETURN
##                the string of the integer value
##
##        _toval(char)
##            converting string to integer
##
##            PARAMS
##                1. char - a String 
##                
##            RETURN
##                the value of the String
##
##        _sector2bearing(sector)
##            converting sectorconverting sector of the robot to an angular value of the robot to an angular value
##
##            PARAMS
##                1. sector - an IR sector (?)
##
##            RETURN
##                the angular value of the sector
##
##        _bearing2sector(bearing)
##            converting bearing of the robot to a sector on the robot
##
##            PARAMS
##                1. bearing - the bearing of the robot
##                
##            RETURN
##                the sector number for the bearing
    

import rone, sys, math

_nbr_state = {}
IR_XMIT_PERIOD_FUDGE = 30

_compute_brg = None
_compute_ornt = None

def init(nbr_period, compute_bearing_func, compute_ornt_func):
    _nbr_state['time_ir_xmit'] = sys.time()
    _nbr_state['time_ir_xmit_offset'] = 0
    _nbr_state['nbr_period'] = nbr_period
    _nbr_state['nbr_timeout'] = 3 * nbr_period
    _nbr_state['message'] = ''
    _nbr_state['xmit_enable'] = True
    _nbr_state['nbr_list'] = []
    _nbr_state['obstacles'] = None
    _nbr_state['obstacles_time'] = sys.time()
    global _compute_brg
    global _compute_ornt
    _compute_brg = compute_bearing_func
    _compute_ornt = compute_ornt_func


       
def _process_nbr_message(ir_msg):
    if ir_msg == None:
        return None
    else:
        (nbr_id, receivers_list, transmitters_list, range) = ir_msg
            
        # compute the bearing from the receivers_list the message was received on
        bearing = _compute_brg(receivers_list)
        
        # compute the orientation from the transmitters_list the message was received on
        orientation = _compute_ornt(transmitters_list)
        return (nbr_id, bearing, orientation, range)



def set_message(message):
    if len(message) == 0:
        _nbr_state['message'] = ''
    else:
        _nbr_state['message'] = message


def get_neighbors():
    nbr_list = []
    for nbr in _nbr_state['nbr_list']:
        nbr_list.append(nbr)
    return nbr_list


def get_obstacles():
    if _nbr_state['obstacles'] == None:
        return None
    else:
        obs_list = []
        for obs in _nbr_state['obstacles']:
            obs_list.append(obs)
        return obs_list


def get_nbr_id(nbr):
    #(nbr_id, message, bearing, orientation, orientation_active, range_bits, msg_time) = nbr
    #return nbr_id
    return nbr[0]

def get_nbr_message(nbr):
    #(nbr_id, message, bearing, orientation, orientation_active, range_bits, msg_time) = nbr
    #return message
    return nbr[1]

def get_nbr_bearing(nbr):
    #(nbr_id, message, bearing, orientation, orientation_active, range_bits, msg_time) = nbr
    #return bearing
    return nbr[2]

def get_nbr_orientation(nbr):
    #(nbr_id, message, bearing, orientation, orientation_active, range_bits, msg_time) = nbr
    #return bearing
    return nbr[3]

def get_nbr_range_bits(nbr):
    #(nbr_id, message, bearing, orientation, orientation_active, range_bits, msg_time) = nbr
    #return range_bits
    return nbr[4]

def nbr_get_update_time(nbr):
    #(nbr_id, message, bearing, orientation, orientation_active, range_bits, msg_time) = nbr
    #return msg_time 
    return nbr[5]


def update():
    current_time = sys.time()
    if current_time < _nbr_state['time_ir_xmit']:
        # not time yet to update the _nbr_state
        return False
    
    _nbr_state['time_ir_xmit'] += _nbr_state['nbr_period']
    while _nbr_state['time_ir_xmit'] < current_time:
        _nbr_state['time_ir_xmit'] += _nbr_state['nbr_period']

    # transmit your announce message
    if _nbr_state['xmit_enable']:
        #The IR message only contains the robot ID
        rone.ir_comms_send_message()
        
        # add a '@' to the front of the radio message to mark it as a nbr message
        rone.radio_send_message('@' + chr(rone.get_id()) + _nbr_state['message'])
        
    # walk over neighbor list and timeout old neighbors
    nbr_list = _nbr_state['nbr_list']
    nbr_idx = 0
    while nbr_idx < len(nbr_list):
        if current_time > (nbr_get_update_time(nbr_list[nbr_idx]) + _nbr_state['nbr_timeout']):
            nbr_list.pop(nbr_idx)
        else:
            nbr_idx += 1

    # time out old obstacles
    if current_time > (_nbr_state['obstacles_time'] + _nbr_state['nbr_timeout']):
        _nbr_state['obstacles'] = None

    # process new messages and update current neighbors
    while True:
        ir_msg = rone.ir_comms_get_message()
        if ir_msg == None:
            break

        (nbr_ID, nbr_bearing, nbr_orientation, nbr_range) = _process_nbr_message(ir_msg)
        #print 'msg recv', nbr_ID            
        if nbr_ID == rone.get_id():
            # this is your own message.  Don't make a neighbor, but process it for obstacles
            (nbr_id, receivers_list, transmitters_list, nbr_range) = ir_msg
            _nbr_state['obstacles'] = receivers_list
            _nbr_state['obstacles_time'] = current_time
            continue

        # this message is from a neighbor.  look for a previous message from this neighbor
        new_nbr = True 
        nbr_idx = 0
        while nbr_idx < len(nbr_list):
            if get_nbr_id(nbr_list[nbr_idx]) == nbr_ID:
                new_nbr = False
                #print 'update nbr ', nbr_ID
                break
            else: 
                nbr_idx += 1
                
        # Add or replace the nbr on the nbr list 
        # note: the order of this tuple is important.  It needs to match the getters above

        if new_nbr:
            nbr = (nbr_ID, '', nbr_bearing, nbr_orientation, nbr_range, current_time)
            nbr_list.append(nbr)
        else:
            nbr_msg = get_nbr_message(nbr_list[nbr_idx])
            nbr = (nbr_ID, nbr_msg, nbr_bearing, nbr_orientation, nbr_range, current_time)
            nbr_list[nbr_idx] = nbr

    #print 'obs',_nbr_state['obstacles']
    
    # update the radio messages every time, in case they arrive out-of-sync with the IR
    while True:
        #Look for neighbor radio messages on the radio queue and if the msg ID is there, make that robot's 
        #msg the incoming message
        #TODO check for '@' char, push other messages back on gueue
        radio_msg = rone.radio_get_message_nbr()
        if radio_msg == None:
            # There are no more radio messages, finished updates
            break
        else:
            # radio_msg[0] = '@', radio_msg[1] = robot ID, radio_msg[2] is the message 
            radio_msg_id = ord(radio_msg[1])
            nbr_idx = 0
            while nbr_idx < len(nbr_list):
                if get_nbr_id(nbr_list[nbr_idx]) == radio_msg_id:
                    #make the radio message the nbr's message
                    radio_msg = radio_msg[2:-1]
                    #print '>',radio_msg,'<'
                    (nbr_ID, old_msg, nbr_bearing, nbr_orientation, nbr_range, current_time) = nbr_list[nbr_idx]
                    nbr = (nbr_ID, radio_msg, nbr_bearing, nbr_orientation, nbr_range, current_time)
                    nbr_list[nbr_idx] = nbr
                    break
                nbr_idx += 1
    return True



def _xmit_enable(val):
    _nbr_state['xmit_enable'] = val
    
def _tochar(val):
    if (val > 255) or (val < 0):
        val = 0
    return chr(val)
    