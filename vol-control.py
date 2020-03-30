import zmq
import time
import sys
from math import pi
from signal import signal, SIGINT
from motor import TalonMotor


def sigint_handle(signal_recieved, frame):
    print("\nProgram interupted. Exiting...")
    exit(0)


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

    # Handle an interupt
    signal(SIGINT, sigint_handle)

    ERROR = 0
    INSPR = 1  # Add air to the patients lungs
    HOLD = 2  # Hold the current position
    OUT = 3  # Back the piston off to refil the air

    states = {ERROR: "ERROR", INSPR: "INSPIRATION",
              HOLD: "HOLDING", OUT: "BREATH_OUT "}

    K_VOL_TO_ENCODER_COUNT = 0.5 * 2.54 * \
        (2.5/2)**2 * pi * 0.01638706 * 1/4096 * 1000  # mL/count of the encoder

    GUI_SETPOINT_INIT = (0.5, 300, 12, 2.0)

    POLL_SUBSCRIBER_TIMEOUT = 100

    PWM_PIN = # TODO
    ENCODER_PIN_A = # TODO
    ENCODER_PIN_B = # TODO
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
    # Setup motor control
    print("Setting up motor control...")
    motor = TalonMotor(PWM_PIN, ENCODER_PIN_A, ENCODER_PIN_B, KP, KI, KD)
    motor.start()
    print("Motor thread started successfully...")

    # Setpoint
    acting_guisetpoint = GUI_SETPOINT_INIT
    new_guisetpoint = GUI_SETPOINT_INIT
    STATE = OUT
    PREV_STATE = HOLD
    state_entry_time = time.time()

    print("Entering state machine...")
    # Run a state machine to create the waveform output we want
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
        Tnoninsp = calcBreathTimePartition(Tinsp, bpm)

        t = time.time() - state_entry_time

        if STATE == INSPR:
            print(
                f"In state {states[STATE]} for {t:3.2f}/{Tinsp} s" + " "*20, end='\r')
            # breath in
            if t > Tinsp:
                motor.controlerEnabled(False)
                motor.setSpeed(0)
                # Change states to hold
                STATE = HOLD
                PREV_STATE = INSPR
                state_entry_time = time.time()
                acting_guisetpoint = new_guisetpoint
            else:
                # Calc and apply motor rate to zero
                # Get the slope
                slope = vol / Tinsp
                motor.setSpeed()

        elif STATE == HOLD:
            # hold current value
            print(
                f"In state {states[STATE]} for {t:3.2f}/{Tnoninsp} s" + " "*20, end='\r')
            if t > Tnoninsp:
                if PREV_STATE == INSPR:
                    motor.controlerEnabled(True)
                    STATE = OUT
                    PREV_STATE = HOLD
                elif PREV_STATE == OUT:
                    motor.controlerEnabled(False)
                    STATE = INSPR
                    PREV_STATE = OUT
                state_entry_time = time.time()

        elif STATE == OUT:
            print(
                f"In state {states[STATE]} for {t:3.2f}/{Tnoninsp} s" + " "*20, end='\r')
            if t > Tnoninsp:
                motor.controlerEnabled(False)
                motor.setSpeed(0)
                STATE = HOLD
                PREV_STATE = OUT
                state_entry_time = time.time()
            else:
                # Calc count position of the motor from vol
                counts = vol/K_VOL_TO_ENCODER_COUNT
                # Set the motor position to the counts
                motor.setPostion(counts)
        elif STATE == ERROR:
            # inform something went wrong
            print("There was an error. Exiting...")
            break

    print("Exit state machine. Program done.")


if __name__ == '__main__':
    main()
