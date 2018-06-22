# train a raspberry pi robot to wander the house while avoiding obstacles


import numpy as np
import RPi.GPIO as gpio
import time
from time import sleep


###############################################################################
# set up hardware
###############################################################################

gpio.setmode(gpio.BOARD)        # use pin numbers not gpio numbers

# ultrasonic sensor
trigger_L = 19
echo_L = 21
trigger_M = 8
echo_M = 10
trigger_R = 16
echo_R = 18


max_distance = 152    # sensor can read 400 cm we only need 5"

gpio.setup(trigger_L, gpio.OUT)
gpio.setup(echo_L, gpio.IN)
gpio.setup(trigger_M, gpio.OUT)
gpio.setup(echo_M, gpio.IN)
gpio.setup(trigger_R, gpio.OUT)
gpio.setup(echo_R, gpio.IN)

# wheels ( 4 wheel motors )
reverse_left = 7
reverse_right = 15
forward_left = 11
forward_right = 13

gpio.setup(reverse_left, gpio.OUT)  
gpio.setup(forward_left, gpio.OUT)  
gpio.setup(forward_right, gpio.OUT) 
gpio.setup(reverse_right, gpio.OUT)

wheel_pulse = 0.5

##############################################################################
# load data from US-015 UltraSonic distance sensor
##############################################################################

distance_from_sensor_to_car_front = 2 * 2.5

# flush sensor
gpio.output(trigger_L, False)
gpio.output(trigger_M, False)
gpio.output(trigger_R, False)
time.sleep(0.5)

# distance to obstacle in path
def get_state(sleep_time=wheel_pulse):

    distances = []
    trigger = [19, 8, 16]
    echo = [21, 10, 18]
    
    for i in range(0,len(trigger)):

        # init 
        pulse_start = 0
        pulse_end = 0
    
        # clear trigger sensor
        gpio.output(trigger[i], False)
        time.sleep(sleep_time)

        # send trigger pulse
        gpio.output(trigger[i], True)
        time.sleep(0.00001)
        gpio.output(trigger[i], False)

        while gpio.input(echo[i]) == 0:
            pulse_start = time.time()

        while gpio.input(echo[i]) == 1:
            pulse_end = time.time()

    
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 343 * 100 / 2.  # speed of sound m/s * m to cm / round trip
    
        if distance > 2 and distance < 400:         # sensor range
            distance = distance + distance_from_sensor_to_car_front

        # don't worry about things further 4'
        # this also reduces the size of the state machine
        if distance >= max_distance:    
            distance = max_distance - 1

        distances.append(int(distance))
        time.sleep(0.1)

    return distances

##############################################################################
# perform action
##############################################################################
#actions = ['forward', 'reverse', 'turn_left', 'turn_right']


def forward(t=wheel_pulse):
    
    gpio.output(forward_right, gpio.HIGH)
    gpio.output(forward_left, gpio.HIGH)
    
    sleep(t)
    gpio.output(forward_right, gpio.LOW)
    gpio.output(forward_left, gpio.LOW)
    

def turn_left(t=wheel_pulse):
    gpio.output(forward_right, gpio.HIGH)
    
    sleep(t)
    gpio.output(forward_right, gpio.LOW)
    
    
def turn_right(t=wheel_pulse):
    gpio.output(forward_left, gpio.HIGH)
    
    sleep(t)
    gpio.output(forward_left, gpio.LOW)
    

def reverse(t=wheel_pulse):
    
    gpio.output(reverse_left, gpio.HIGH)
    gpio.output(reverse_right, gpio.HIGH)
    
    sleep(t)
    gpio.output(reverse_left, gpio.LOW)
    gpio.output(reverse_right, gpio.LOW)

    

##########################################################################
# cleanup
##########################################################################

def cleanup():

    gpio.cleanup()


##########################################################################
# world
##########################################################################

class world():

    def __init__(self):
        
        self.state = get_state()
        #self.states = np.zeros(max_distance + 1)

    def move(self, action):

        state = get_state()
        reward = 0


        # penatly for being too closes to an obstacle
        if not (all(int(i) <= 12 for i in state)):   # buffer zone converted to cm
            reward = -50.0
            #reverse()

        if action == 0:
            turn_left()
            reward = 1
        elif action == 1:
            turn_right()
            reward = 1
        elif action == 2:       
            forward()
            reward = 1
        
        
        #print("state %d,  action %d,  reward %d" % (state, action, reward))
        
        return reward,state



if __name__ == "__main__":
    world = world()
    reward,state = world.move(2)
    print reward,state
    cleanup()
 
