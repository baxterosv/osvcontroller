import zmq


def main():

    # Some constants
    O2_DEADBAND_SYM = 0.05

    # Setup ZeroMQ
    ctxt = zmq.Context()
    setpntsub = ctxt.socket(zmq.SUB)
    setpntsub.bind("ipc:///tmp/gui_setpoint.pipe")

    o2datapub = ctxt.socket(zmq.PUB)
    o2datapub.connect("ipc:///tmp/o2_data.pipe")

    poller = zmq.Poller()
    poller.register(setpntsub, zmq.POLLIN)

    # Setup O2 sensor read

    # Setpoint
    guisetpoint = None

    while True:  # Add way to escape

        # poll the setpoint sub
        socks = dict(poller.poll())
        if setpntsub in socks:
            guisetpoint = setpntsub.recv_pyobj()

        # read the o2 data
        o2sensorval = read_o2()

        if guisetpoint != None:
            o2setpoint = guisetpoint[0]
            # Leaves 10% symmetric deadband around setpoint
            if o2setpoint - O2_DEADBAND_SYM > o2sensorval:
                pass # open solenoid
            elif o2setpoint + O2_DEADBAND_SYM < o2sensorval:
                pass # close solenoid
        
        # publish o2 data
        o2datapub.send_pyobj(o2sensorval)

if __name__ == '__main__':
    main()
