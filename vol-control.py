import zmq
import time
from math import pi

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

    ERROR = 0
    INSPR = 1 # Add air to the patients lungs
    HOLD = 2 # Hold the current position
    OUT = 3 # Back the piston off to refil the air

    K_VOL_TO_ENCODER_COUNT = 0.5 * 2.54 * (2.5/2)**2 * pi * 0.01638706 * 1/4096 * 1000 # mL/count of the encoder

    GUI_SETPOINT_INIT = (0.5, 300, 12, 2.0)

    # Setup ZeroMQ
    ctxt = zmq.Context()
    setpntsub = ctxt.socket(zmq.SUB)
    setpntsub.bind("ipc:///tmp/gui_setpoint.pipe")

    voldatapub = ctxt.socket(zmq.PUB)
    voldatapub.connect("ipc:///tmp/vol_data.pipe")

    poller = zmq.Poller()
    poller.register(setpntsub, zmq.POLLIN)

    # Setup motor control

    # Setpoint
    acting_guisetpoint = GUI_SETPOINT_INIT
    new_guisetpoint = GUI_SETPOINT_INIT
    STATE = OUT
    PREV_STATE = HOLD
    state_entry_time = time.time()

    # Run a state machine to create the waveform output we want
    while True:  # TODO Add a way to escape

        # Poll the subscriber for new setpoints
        socks = dict(poller.poll())
        if setpntsub in socks:
            new_guisetpoint = setpntsub.recv_pyobj()

        # Unpack the setpoints
        _, vol, bpm, Tinsp = acting_guisetpoint

        # Calculate the time partition for the non insp states
        Tnoninsp = calcBreathTimePartition(Tinsp, bmp)

        if STATE == INSPR:
            #breath in
            if time.time() - state_entry_time > Tinsp:
                # Change states to hold
                STATE = HOLD
                PREV_STATE = INSPR
                state_entry_time = time.time()
            else:
                # Calc and apply motor action
                # Get the slope
                slope = vol / Tinsp
                t = time.time() - state_entry_time
                setMotorPosition(K_VOL_TO_MOTOR_POS * slope * t)

        elif STATE == HOLD:
            #hold current value
            if time.time() - state_entry_time > Tnoninsp:
                STATE = OUT
                state_entry_time = time.time()
        elif STATE == OUT:
            if time.time() - state_entry_time > Tnoninsp:
                STATE = HOLD
                PREV_STATE = OUT
                state_entry_time = time.time()
                acting_guisetpoint = new_guisetpoint
            else:
                # Calc count position of the motor from vol
                counts = vol/K_VOL_TO_ENCODER_COUNT
                # Set the motor position to the counts
                motor.setPositionCounts(counts)
                
        elif STATE == ERROR:
            #inform something went wrong
            pass


if __name__ == '__main__':
    main()
