import zmq
import time
import sys
from math import pi
from signal import signal, SIGINT
from motor2 import TalonMotor
import RPi.GPIO as GPIO


def sigint_handle(signal_recieved, frame):
    print("\nProgram interupted. Exiting...")
    exit(0)


def calcBreathTimePartition(inhalation_period, bpm):
    """Calculates the time for non inpiration states. Assumes remaining time is divided equally amoung the stages.

    Arguments:
        inhalation_period {float} -- Inspiration period in seconds
        bpm {float} -- breaths per minute

    Returns:
        float -- non-Inspiration period in seconds
    """
    breath_period = 1/bpm * 60.0
    exhalation_period = breath_period - inhalation_period
    return exhalation_period


def main():

    # Handle an interupt
    signal(SIGINT, sigint_handle)

    ERROR = 0
    INSPR = 1  # Add air to the patients lungs
    #HOLD = 2  # Hold the current position -- removed
    EXHAL = 2  # Back the piston off to refil the air

    states = {ERROR: "ERROR", INSPR: "INSPIRATION", EXHAL: "EXHALATION "}
    
    #control position of piston using ml only
    GUI_SETPOINT_INIT = (0.5, 100, 15, 2) # 02, vol, BPM, Inspiration rate 

    PITCH = 0.5 #cm/rotation
    BORE_DIAMETER = 2.5 #inches
    ML_PER_CUBIC_INCH = 16.3871 #ml/cubic inch
    MAX_DISTANCE = 6 #inches, maximum movement of bore
    CM_TO_INCH = 1.0/2.54 #cm to inch
    COUNTS_PER_REV = 20 #encoder counts per revolution, important to update given different encoders
    
    
    K_VOL_TO_ENCODER_COUNT = PITCH*CM_TO_INCH*((BORE_DIAMETER/2)**2*pi)*ML_PER_CUBIC_INCH/COUNTS_PER_REV #mL/count of the encoder
    MAX_ENCODER_COUNT = K_VOL_TO_ENCODER_COUNT*((BORE_DIAMETER/2)**2*pi)*MAX_DISTANCE*ML_PER_CUBIC_INCH #maximum counts for volume of one cylinder
    print(MAX_ENCODER_COUNT)

    POLL_SUBSCRIBER_TIMEOUT = 100

    PWM_PIN = 13
    ENCODER_PIN_A = 21
    ENCODER_PIN_B = 20
    END_STOP_PIN = 18
    HALL_PIN = 22
    KP = 1.0 # NOTE Can change this
    KI = 0 # NOTE Not implemented
    KD = 0 # NOTE Not implemented

    # Setup ZeroMQ
    print("ZeroMQ init...")
    ctxt = zmq.Context()
    setpntsub = ctxt.socket(zmq.SUB)
    setpntsub.bind("ipc:///tmp/gui_setpoint.pipe")

    voldatapub = ctxt.socket(zmq.PUB)
    voldatapub.connect("ipc:///tmp/vol_data.pipe")

    poller = zmq.Poller()
    poller.register(setpntsub, zmq.POLLIN)
    print("ZeroMQ finished init...")
    
    # Setup motor controlHOLD
    print("Setting up motor control...")
    motor = TalonMotor(PWM_PIN, ENCODER_PIN_A, ENCODER_PIN_B, KP, KI, KD,K_VOL_TO_ENCODER_COUNT, GUI_SETPOINT_INIT[1],END_STOP_PIN,HALL_PIN)
    motor.start()
    print("Motor thread started successfully...")

    # Setpoint
    acting_guisetpoint = GUI_SETPOINT_INIT
    new_guisetpoint = GUI_SETPOINT_INIT
    STATE = EXHAL
    PREV_STATE = INSPR
    state_entry_time = time.time()


    print("Entering state machine...")
    # Run a state machine to create the waveform output we want
    try:
        while True:  # TODO Add a way to escape

            # Poll the subscriber for new setpoints
            # NOTE Timeout is set in constants
            socks = dict(poller.poll(POLL_SUBSCRIBER_TIMEOUT))
            if setpntsub in socks:
                print("!! Recieved new setpoint !!")
                new_guisetpoint = setpntsub.recv_pyobj()

            # Unpack the setpoints
            _, vol, bpm, Tinsp = acting_guisetpoint

            # Calculate the time partition for the non insp states
            Texhal = calcBreathTimePartition(Tinsp, bpm)
            
            #max inhalation and exhalation speeeds
            maxInsprSpeed = 0.8
            maxExhalSpeed = 0.8
            

            t = time.time() - state_entry_time

            if STATE == INSPR:
                #print(
                #    f"In state {states[STATE]} for {t:3.2f}/{Tinsp} s" + " "*20, end='\r')
                print("\nInspiration")
                # breath in
                if t > Tinsp:
                    # Change states to hold
                    STATE = EXHAL
                    PREV_STATE = INSPR
                    state_entry_time = time.time()
                    acting_guisetpoint = new_guisetpoint
                else:
                    # Calc and apply motor action
                    # Get the slopevol
                    # slope = vol / Tinsp
                    if (t < 0.1 and motor.limitHit(0)):
                        motor.setSpeed(0)
                    else:
                        #motor.setVolume(0,vol,maxInsprSpeed)
                        motor.setSpeed(5.5)
                    print(f"Position: {motor.encoder.position()}")
                    print(f"Speed: {motor.speed}")

            elif STATE == EXHAL:
                #print(
                #   f"In state {states[STATE]} for {t:3.2f}/{Texhal} s" + " "*20, end='\r')
                print("\nExhalation")
                if t > Texhal:
                    STATE = INSPR
                    PREV_STATE = EXHAL
                    state_entry_time = time.time()
                else:
                    if (t > 0.1 and motor.limitHit(vol)):
                        motor.setSpeed(0)
                    else:
                    # Set the motor position to the counts
                        #motor.setVolume(vol,vol,maxExhalSpeed)
                        motor.setSpeed(8)
                    print(f"Position: {motor.encoder.position()}")
                    print(f"Speed: {motor.speed}")
                    
            elif STATE == ERROR:
                # inform something went wrong
                print("There was an error. Exiting...")
                motor.stop()
                break
    except KeyboardInterrupt:
        print("\nYou've exited the program\n")
    
    finally:
        print("Exit state machine. Cleanup.")
        GPIO.cleanup()

if __name__ == '__main__':
    main()
