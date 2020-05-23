import zmq
import time
import sys
from math import pi
from signal import signal, SIGINT
import RPi.GPIO as GPIO
from smbus import SMBus
from enum import Enum

from roboclaw_3 import Roboclaw

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from zmqTopics import *


# State enumeration
class State(Enum):
    ERROR = 0       # Exit the program with a code
    INSPR = 1       # Add air to the patients lungs
    HOLD = 2        # Hold the current position
    OUT = 3         # Back the piston off to refil the air
    STOPPED = 4     # Wait for GUI start signal

''' CONSTANTS '''
# State to string mappings for printing
STATES = {State.ERROR:"ERROR", State.INSPR:"INSPIRATION", State.HOLD:"HOLDING", State.OUT:"BREATH_OUT", State.STOPPED:"STOPPED"}

# Geometric quantities for control
PITCH = 7.0874333333  # in/rotation
BORE_DIAMETER = 2.5  # inches
ML_PER_CUBIC_INCH = 16.3871  # ml/cubic inch
MAX_DISTANCE = 6  # inches, maximum movement of bore
CM_TO_INCH = 1.0/2.54  # cm to inch
# encoder counts per revolution, important to update given different encoders
COUNTS_PER_REV = 4096
K_VOL_TO_ENCODER_COUNT = PITCH * \
    ((BORE_DIAMETER/2)**2*pi)*ML_PER_CUBIC_INCH / \
    COUNTS_PER_REV  # mL/count of the encoder


MAX_ENCODER_COUNT = 6500 #max number of counts

END_STOP = 24 #endstop
END_STOP_MARGIN = 50 #margin to move back after hitting the endstop

# Control gains NOTE not used right now
KP = 1.0
KI = 0
KD = 0

# ZMQ settings
ZMQ_POLL_SUBSCRIBER_TIMEOUT_MS = 100   # wait up to this long for response
ZMQ_GUI_TOPIC = "ipc:///tmp/gui_setpoint.pipe"
ZMQ_HEARTBEAT_INTERVAL_SEC = 1         # expected time between heartbeat
ZMQ_MEASUREMENT_TOPIC = "ipc:///tmp/vol_data.pipe"

# Roboclaw settings
# NOTE for instance... change based on your wiring
ROBOCLAW_COMPORT = "/dev/ttyS0"
ROBOCLAW_BAUDRATE = 38400
ROBOCLAW_ADDRESS = 0x80  # Set in Motion Studio
# NOTE increase to make acceleration of motor more agressive
ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS = 1.5  # units: s^-1

#Flow Sensor settings
FLOW_SENSOR_ADDRESS = 0x6C # Try 0x6F if this doesn't work
SENSOR_MEASUREMENT_WAIT_TIME = 0.120 # s
FLOW_SENSOR_RANGE = 50.0
START_BITS = [0xD0, 0x40, 0x18, 0x06] #from datasheet, bits to write to start sensor
FLOW_BITS = [0xD0, 0x51, 0x2C, 0x07] #from datasheet, bits to read flow

#Pressure sensor headings
PRESSURE_SENSOR_ADDRESS = 0x28 # Found using i2cdetect -y 1 on Raspberry Pi
SENSOR_MEASUREMENT_WAIT_TIME = 0.120 # s
SENSOR_COUNT_MIN = 1638.3 #from datasheet, starting at 10% on bottom
SENSOR_COUNT_MAX = 14744.7 #from datasheet, ending at 90% on top
SENSOR_PRESSURE_MIN = -1.0 #from datasheet
SENSOR_PRESSURE_MAX = 1.0 #from datasheet
INIT_SIGNAL = 0x01 #init signal fori2c


def sigint_handle(signal_recieved, frame):
    """Handles a CNTL+C from the user. Should exit gracefully.
    """
    # TODO Need to kill the motor here and clean up
    print("\nProgram interupted. Exiting...")
    exit(0)
    
def endStop_handler(motor):
    print("End Stop Handler\n")
    motor.ResetEncoders(ROBOCLAW_ADDRESS)
    motor.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, 500, 500,500, -END_STOP_MARGIN, 0)
    time.sleep(0.5)
    motor.ResetEncoders(ROBOCLAW_ADDRESS)

def calcBreathTimePartition(inspiration_period, bpm):
    """Calculates the time for non inpiration states. Assumes remaining time is divided equally amoung the stages.

    Arguments:
        inspiration_period {float} -- inspiration period in seconds
        bpm {float} -- breaths per minute

    Returns:
        float -- non-inspiration period in seconds
    """
    breath_period = 1/bpm * 60.0
    non_inspiration_period = abs(inspiration_period - breath_period) / 3
    return non_inspiration_period

def calcVolume(bus):
    #NOTE -- may need to add a signal delay before calling this in the loop
    # Start the sensor - [D040] <= 0x06
    bus.write_i2c_block_data(FLOW_SENSOR_ADDRESS, 0x00,START_BITS)
    
    # Tell the sensor we want to read the flow value [D051/D052] => Read Compensated Flow value
    bus.write_i2c_block_data(FLOW_SENSOR_ADDRESS, 0x00, FLOW_BITS)

    # Read the values
    r = bus.read_i2c_block_data(FLOW_SENSOR_ADDRESS, 0x07, 2)
    i = int.from_bytes(r, byteorder='big') # NOTE: try 'big' if not working

    # Do the conversion
    rd_flow = ((i -  1024.0) * FLOW_SENSOR_RANGE / 60000.0) # convert to [L/min](x10)
    return rd_flow

def calcPressure(bus):
    #NOTE -- may need to add a signal delay before calling this in the loop
    # Tell the sensor we want to read the flow value [D051/D052] => Read Compensated Flow value
    answer = bus.read_word_data(PRESSURE_SENSOR_ADDRESS,INIT_SIGNAL)
    
    #bit  shift to get the full value
    answer=float((((answer&0x00FF)<< 8) + ((answer&0xFF00) >> 8)))
    pressure = (answer-SENSOR_COUNT_MIN)*(SENSOR_PRESSURE_MAX- SENSOR_PRESSURE_MIN)/(SENSOR_COUNT_MAX-SENSOR_COUNT_MIN) + SENSOR_PRESSURE_MIN
    return round(pressure,2)

def calcOxygen(chan):
    return chan.voltage

def main():

    # Setup SIGINT handler
    signal(SIGINT, sigint_handle)

    # Setup ZeroMQ
    print("Initializing ZeroMQ...")
    ctxt = zmq.Context()
    setpntsub = ctxt.socket(zmq.SUB)
    setpntsub.bind(ZMQ_CONTROL_SETPOINTS)
    setpntsub.setsockopt_string(zmq.SUBSCRIBE, '')

    graph_data = ctxt.socket(zmq.PUB)
    graph_data.connect(ZMQ_GRAPH_DATA_TOPIC)

    

    poller = zmq.Poller()
    poller.register(setpntsub, zmq.POLLIN)
    print("ZeroMQ finished init...")

    # Setup motor control
    print("Setting up motor control...")
    motor = Roboclaw(ROBOCLAW_COMPORT, ROBOCLAW_BAUDRATE)
    r = motor.Open()
    if r < 1:
        print("Error: could not connect to Roboclaw...")
        print("Exit...")
        exit(0)
    print("Motor thread started successfully...")

    #Setup i2C bus
    bus = SMBus(1) #create I2C bus
    bus_ADC = busio.I2C(board.SCL, board.SDA)
    
    #Setup flow sensor
    bus.write_byte_data(FLOW_SENSOR_ADDRESS, 0x0B, 0x00) #initialize the I2C device

    #Setup oxygen sensor
    # Create the I2C bus
    #i2c = busio.I2C(board.SCL, board.SDA)

    # Create the ADC object using the I2C bus
    ads = ADS.ADS1115(bus_ADC)
    # you can specify an I2C adress instead of the default 0x48
    # ads = ADS.ADS1115(i2c, address=0x49)

    # Create single-ended input on channel 3
    chan = AnalogIn(ads, ADS.P3)


    # Setup End Stop
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(END_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Usually High (True), Low (False) when triggered
    # GPIO.add_event_detect(END_STOP, GPIO.RISING, callback= lambda x: endStop_handler(motor), bouncetime=200) 

    #### MOVED TO STOPPED, start in that state ####
    # # Wait for first setpoint from GUI
    # print("Waiting for initial setpoint from GUI...")
    # socks = dict()
    # while setpntsub not in socks:
    #     socks = dict(poller.poll(10 * ZMQ_POLL_SUBSCRIBER_TIMEOUT_MS))
    # print("Recieved intial setpoint from GUI!")
    # sp = setpntsub.recv_pyobj()
    # print(f"The intial setpoint is {sp}")
    # acting_guisetpoint = sp  # Currently acting setpoint
    # new_guisetpoint = sp  # New setpoint set only when leaving INSPR of OUT states

    # #intialize motor setpoint to 0
    # print("Initiializing 0 location...")
    # motor.SetEncM1(ROBOCLAW_ADDRESS,-MAX_ENCODER_COUNT) #set current loaction to maximum pull possible
    # while  GPIO.input(END_STOP):
    #     motor.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, 500, 250,500, 0, 0)
    # motor.ResetEncoders(ROBOCLAW_ADDRESS)
    # motor.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, 500, 500,500, -END_STOP_MARGIN, 0)
    # time.sleep(1)
    # motor.ResetEncoders(ROBOCLAW_ADDRESS)
    # time.sleep(1)   
    
    # # Initial states for state machine
    # state = State.OUT
    # prev_state = State.HOLD
    # state_entry_time = time.time()
    state = State.STOPPED
    prev_state = State.STOPPED
    
    #Initialize guisetpoint
    acting_guisetpoint =  [0,0,0,0,True]


    print("Entering state machine...")
    # Run a state machine to create the waveform output we want
    while True:  # TODO Add a way to escape

        # Poll the subscriber for new setpoints
        # NOTE Timeout is set in constants
        socks = dict(poller.poll(ZMQ_POLL_SUBSCRIBER_TIMEOUT_MS))
        if setpntsub in socks:
            new_guisetpoint = setpntsub.recv_pyobj()
            t_recent_heartbeat = time.time()
            print(f"Recieved a new setpoint of {new_guisetpoint}" + " "*20)

        # Unpack the setpoints
        _, vol, bpm, Tinsp, Stopped = acting_guisetpoint

        # Check if stop requested
        if Stopped:
            state = State.STOPPED
        # If 3 heartbeats missed, go to stopped
        elif (time.time() - t_recent_heartbeat) >= (ZMQ_HEARTBEAT_INTERVAL_SEC * 3):
            Stopped = True
            state = State.STOPPED
        # Heartbeat and not stopped, then update
        else:
            # Calculate the time partition for the non insp states
            Tnoninsp = calcBreathTimePartition(Tinsp, bpm)

            #Calculate flow from device
            flow = calcVolume(bus)
            #flow = 0

            #Calculate pressure from device
            pressure = calcPressure(bus)
            
            # Calculate O2% from device
            oxygen = calcOxygen(chan)
            
            #send data back to GUI
            gui_data = (pressure,flow,oxygen)
            print(gui_data)
            # Build and send message from current values
            graph_data.send_pyobj(gui_data)
            
            # Calculate time we have been in this state so far
            t = time.time() - state_entry_time
            


        if state == State.INSPR:
            print(
                f"In state {STATES[state]} for {t:3.2f}/{Tinsp} s" + " "*20, end='\r')
            # breath in
            if t > Tinsp:
                # TODO Stop motor movement
                # Change states to hold
                state = State.HOLD
                prev_state = State.INSPR
                state_entry_time = time.time()
                acting_guisetpoint = new_guisetpoint
            else:
                # Calc and apply motor rate to zero
                # Get the slope
                slope = vol / Tinsp
                slope_encoder = int(slope / K_VOL_TO_ENCODER_COUNT)
                accel_encoder = int(slope_encoder * ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS)
                motor.SpeedAccelDeccelPositionM1(
                    ROBOCLAW_ADDRESS, accel_encoder, slope_encoder, accel_encoder, 0, 0)
        elif state == State.HOLD:
            # hold current value
            print(
                f"In state {STATES[state]} for {t:3.2f}/{Tnoninsp} s" + " "*20, end='\r')
            if t > Tnoninsp:
                if prev_state == State.INSPR:
                    state = State.OUT
                    prev_state = State.HOLD
                elif prev_state == State.OUT:
                    state = State.INSPR
                    prev_state = State.OUT
                state_entry_time = time.time()
        elif state == State.OUT:
            print(
                f"In state {STATES[state]} for {t:3.2f}/{Tnoninsp} s" + " "*20, end='\r')
            if t > Tnoninsp:
                state = State.HOLD
                prev_state = State.OUT
                state_entry_time = time.time()
            else:
                # Calc count position of the motor from vol
                counts = int(vol/K_VOL_TO_ENCODER_COUNT)
                speed_counts = int(counts / Tnoninsp * 1.1)  # A little faster for wiggle room
                accel_counts = int(speed_counts * ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS)
                #print(f"Counts: {counts}, Speed: {speed_counts}, Accel: {accel_counts}, Aggressive: {ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS}")
                #print(motor.ReadISpeedM1(ROBOCLAW_ADDRESS))
                motor.SpeedAccelDeccelPositionM1(
                    ROBOCLAW_ADDRESS, accel_counts, speed_counts, accel_counts, -counts, 0)
        elif state == State.STOPPED:
            # Set speed of motor to 0
            motor.ForwardM1(ROBOCLAW_ADDRESS, 0)

            print(f"Stopped!")
            print(f"In state {STATES[state]}, waiting for start signal from GUI...")
            
            # Wait for start signal from GUI
            while Stopped:

                socks = dict()
                while setpntsub not in socks:
                    socks = dict(poller.poll(10 * ZMQ_POLL_SUBSCRIBER_TIMEOUT_MS))
                

                sp = setpntsub.recv_pyobj()
                t_recent_heartbeat = time.time()
                _,_,_,_,Stopped = sp

                if Stopped:
                    print("Heartbeat recieved, waiting for start signal from GUI...")

            print(f"Recieved start signal from GUI, with setpoint {sp}")
            acting_guisetpoint = sp  # Currently acting setpoint
            new_guisetpoint = sp  # New setpoint set only when leaving INSPR of OUT states
            
            #intialize motor setpoint to 0
            print("Initiializing 0 location...")
            motor.SetEncM1(ROBOCLAW_ADDRESS,-MAX_ENCODER_COUNT) #set current loaction to maximum pull possible
            while  GPIO.input(END_STOP):
                motor.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, 500, 250,500, 0, 0)
            motor.ResetEncoders(ROBOCLAW_ADDRESS)
            motor.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, 500, 500,500, -END_STOP_MARGIN, 0)
            time.sleep(1)
            motor.ResetEncoders(ROBOCLAW_ADDRESS)
            time.sleep(1)   
            
            # Initial states for state machine
            state = State.OUT
            prev_state = State.HOLD
            state_entry_time = time.time()
            print("Beginning to breath!")

        elif state == State.ERROR:
            # inform something went wrong
            print("There was an error. Exiting...")
            break

    print("Exit state machine. Program done.")


if __name__ == '__main__':
    main()
