import zmq
import time

def calcBreathTimePartition(inspiration_period, bpm):
    breath_period = 1/bpm * 60.0
    non_inspiration_period = abs(inspiration_period - breath_period) / 3
    return non_inspiration_period

def main():

    ERROR = 0
    INSPR = 1
    HOLD = 2
    OUT = 3

    K_VOL_TO_MOTOR_POS = 0.0

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
    guisetpoint = None
    STATE = INSPR
    state_entry_time = time.time()

    while True:  # Add a way to escape

        socks = dict(poller.poll())
        if setpntsub in socks:
            guisetpoint = setpntsub.recv_pyobj()

        _, vol, bpm, Tinsp = guisetpoint
        Tnoninsp = calcBreathTimePartition(Tinsp, bmp)

        # State machine for volume control
        if STATE == INSPR:
            #breath in
            if time.time() - state_entry_time > Tinsp:
                # Change states to hold
                STATE = HOLD
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
            #let breath out (piston retract)
            pass
        elif STATE == ERROR:
            #inform something went wrong
            pass


if __name__ == '__main__':
    main()
