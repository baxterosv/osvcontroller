import zmq
import time
import sys
from math import pi
from signal import signal, SIGINT
import RPi.GPIO as GPIO

from roboclaw_3 import Roboclaw

''' CONSTANTS '''
# State enumeration
ERROR = 0  # Exit the program with a code
INSPR = 1  # Add air to the patients lungs
HOLD = 2  # Hold the current position
OUT = 3  # Back the piston off to refil the air
# State to string mappings for printing
STATES = {ERROR: "ERROR", INSPR: "INSPIRATION",
          HOLD: "HOLDING", OUT: "BREATH_OUT "}

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
ZMQ_POLL_SUBSCRIBER_TIMEOUT = 100
ZMQ_GUI_TOPIC = "ipc:///tmp/gui_setpoint.pipe"
ZMQ_MEASUREMENT_TOPIC = "ipc:///tmp/vol_data.pipe"

# Roboclaw settings
# NOTE for instance... change based on your wiring
ROBOCLAW_COMPORT = "/dev/ttyS0"
ROBOCLAW_BAUDRATE = 38400
ROBOCLAW_ADDRESS = 0x80  # Set in Motion Studio
# NOTE increase to make acceleration of motor more agressive
ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS = 1.5  # units: s^-1


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


def main():

    # Setup SIGINT handler
    signal(SIGINT, sigint_handle)

    # Setup ZeroMQ
    print("Initializing ZeroMQ...")
    ctxt = zmq.Context()
    setpntsub = ctxt.socket(zmq.SUB)
    setpntsub.bind(ZMQ_GUI_TOPIC)
    setpntsub.setsockopt_string(zmq.SUBSCRIBE, '')

    voldatapub = ctxt.socket(zmq.PUB)
    voldatapub.connect(ZMQ_MEASUREMENT_TOPIC)

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

    # Wait for first setpoint from GUI
    print("Waiting for initial setpoint from GUI...")
    socks = dict()
    while setpntsub not in socks:
        socks = dict(poller.poll(10 * ZMQ_POLL_SUBSCRIBER_TIMEOUT))
    print("Recieved intial setpoint from GUI!")
    sp = setpntsub.recv_pyobj()
    print(f"The intial setpoint is {sp}")
    acting_guisetpoint = sp  # Currently acting setpoint
    new_guisetpoint = sp  # New setpoint set only when leaving INSPR of OUT states

    #intialize motor setpoint to 0
    
    motor.SetEncM1(ROBOCLAW_ADDRESS,-MAX_ENCODER_COUNT) #set current loaction to maximum pull possible
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(END_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("Initiializing 0 location...")
    
    while  GPIO.input(END_STOP):
        motor.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, 500, 250,500, 0, 0)
        #print(motor.ReadEncM1(ROBOCLAW_ADDRESS))
    #motor.SpeedM1(ROBOCLAW_ADDRESS,0)
    motor.ResetEncoders(ROBOCLAW_ADDRESS)
    motor.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS, 500, 500,500, -END_STOP_MARGIN, 0)
    time.sleep(1)
    motor.ResetEncoders(ROBOCLAW_ADDRESS)
    time.sleep(1)
    print(motor.ReadEncM1(ROBOCLAW_ADDRESS))
    
    
    #GPIO.add_event_detect(END_STOP, GPIO.RISING, callback= lambda x: endStop_handler(motor), bouncetime=200) 
    
    # Initial states for state machine
    state = OUT
    prev_state = HOLD
    state_entry_time = time.time()

    print("Entering state machine...")
    # Run a state machine to create the waveform output we want
    while True:  # TODO Add a way to escape

        # Poll the subscriber for new setpoints
        # NOTE Timeout is set in constants
        socks = dict(poller.poll(ZMQ_POLL_SUBSCRIBER_TIMEOUT))
        if setpntsub in socks:
            new_guisetpoint = setpntsub.recv_pyobj()
            print(f"Recieved a new setpoint of {new_guisetpoint}" + " "*20)

        # Unpack the setpoints
        _, vol, bpm, Tinsp = acting_guisetpoint

        # Calculate the time partition for the non insp states
        Tnoninsp = calcBreathTimePartition(Tinsp, bpm)

        
        
        # Calculate time we have been in this state so far
        t = time.time() - state_entry_time

        if state == INSPR:
            print(
                f"In state {STATES[state]} for {t:3.2f}/{Tinsp} s" + " "*20, end='\r')
            # breath in
            if t > Tinsp:
                # TODO Stop motor movement
                # Change states to hold
                state = HOLD
                prev_state = INSPR
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
        elif state == HOLD:
            # hold current value
            print(
                f"In state {STATES[state]} for {t:3.2f}/{Tnoninsp} s" + " "*20, end='\r')
            if t > Tnoninsp:
                if prev_state == INSPR:
                    state = OUT
                    prev_state = HOLD
                elif prev_state == OUT:
                    state = INSPR
                    prev_state = OUT
                state_entry_time = time.time()
        elif state == OUT:
            print(
                f"In state {STATES[state]} for {t:3.2f}/{Tnoninsp} s" + " "*20, end='\r')
            if t > Tnoninsp:
                state = HOLD
                prev_state = OUT
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
        elif state == ERROR:
            # inform something went wrong
            print("There was an error. Exiting...")
            break

    print("Exit state machine. Program done.")


if __name__ == '__main__':
    main()
