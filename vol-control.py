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

from threading import Thread, Event

from operator import itemgetter

import logging
logging.basicConfig(level=logging.INFO)

class TimeManagedList():
    def __init__(self, period=10.0):
        self.period = period
        self.l = []
    def getMax(self) -> float:
        return max(self.l, key=itemgetter(1))[1]
    def getMin(self) -> float:
        return min(self.l, key=itemgetter(1))[1]
    def append(self, data, time=0):
        self.l.append((time, data))
    def update(self, time=0):
        current_time = time
        self.l = [a for a in self.l if current_time - a[0] < self.period]
    def setPeriod(self, period):
        self.period = period
    def integrate(self):
        i = 0
        for a in range(1, len(self.l)):
            tu = self.l[a][0]
            tl = self.l[a-1][0]
            vu = self.l[a][1]
            vl = self.l[a-1][1]
            delt = (tu - tl) * (vu + vl) / 2
            i = i + delt
        return i


# Alarm enumeration
class Alarm(Enum):
    NONE = 0 # No Alarms Present
    TRIGGERED = 1 # Alarm currently triggered
    SUPPRESSED = 2 # Alarm currently suppressed

# State enumeration
class State(Enum):
    ERROR = 0       # Exit the program with a code
    INSPR = 1       # Add air to the patients lungs
    HOLD = 2        # Hold the current position
    OUT = 3         # Back the piston off to refil the air
    STOPPED = 4     # Wait for GUI start signal

class OSVController(Thread):

    def __init__(self):
        super().__init__()

        ''' CONSTANTS '''
        # State to string mappings for printing
        self.STATES = {State.ERROR:"ERROR", State.INSPR:"INSPIRATION", State.HOLD:"HOLDING", State.OUT:"BREATH_OUT", State.STOPPED:"STOPPED"}

        # Geometric quantities for control
        self.PITCH = 7.0874333333  # in/rotation
        self.BORE_DIAMETER = 2.5  # inches
        self.ML_PER_CUBIC_INCH = 16.3871  # ml/cubic inch
        self.MAX_DISTANCE = 6  # inches, maximum movement of bore
        self.CM_TO_INCH = 1.0/2.54  # cm to inch
        # encoder counts per revolution, important to update given different encoders
        self.COUNTS_PER_REV = 4096
        self.K_VOL_TO_ENCODER_COUNT = self.PITCH * \
            ((self.BORE_DIAMETER/2)**2*pi)*self.ML_PER_CUBIC_INCH / \
            self.COUNTS_PER_REV  # mL/count of the encoder


        self.MAX_ENCODER_COUNT = 6500 #max number of counts

        self.HALL_EFFECT_SENSOR = 24 # Hall-effect sensor
        self.END_STOP_MARGIN = 50 #margin to move back after hitting the endstop

        # Control gains NOTE not used right now
        self.KP = 1.0
        self.KI = 0
        self.KD = 0

        self.DIR_UP = 1
        self.DIR_DOWN = -1

        # ZMQ settings
        self.ZMQ_POLL_SUBSCRIBER_TIMEOUT_MS = 100   # wait up to this long for response
        self.ZMQ_HEARTBEAT_INTERVAL_SEC = 1         # expected time between heartbeat

        # Roboclaw settings
        # NOTE for instance... change based on your wiring
        self.ROBOCLAW_COMPORT = "/dev/ttyS0"
        self.ROBOCLAW_BAUDRATE = 38400
        self.ROBOCLAW_ADDRESS = 0x80  # Set in Motion Studio
        # NOTE increase to make acceleration of motor more agressive
        self.ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS = 1.5  # units: s^-1

        #Flow Sensor settings
        self.FLOW_SENSOR_ADDRESS = 0x6C # Try 0x6F if this doesn't work
        self.SENSOR_MEASUREMENT_WAIT_TIME = 0.120 # s
        self.FLOW_SENSOR_RANGE = 50.0
        self.START_BITS = [0xD0, 0x40, 0x18, 0x06] #from datasheet, bits to write to start sensor
        self.FLOW_BITS = [0xD0, 0x51, 0x2C, 0x07] #from datasheet, bits to read flow

        #Pressure sensor headings
        self.PRESSURE_SENSOR_ADDRESS = 0x28 # Found using i2cdetect -y 1 on Raspberry Pi
        self.SENSOR_MEASUREMENT_WAIT_TIME = 0.120 # s
        self.SENSOR_COUNT_MIN = 1638.3 #from datasheet, starting at 10% on bottom
        self.SENSOR_COUNT_MAX = 14744.7 #from datasheet, ending at 90% on top
        self.SENSOR_PRESSURE_MIN = -1.0 #from datasheet
        self.SENSOR_PRESSURE_MAX = 1.0 #from datasheet
        self.INIT_SIGNAL = 0x01 #init signal fori2c

        self.quitEvent = Event()
        self.hallEffectEvent = Event()

        self.pressure_list = TimeManagedList()
        self.volume_list = TimeManagedList()

        self.tidal_volume = None

        # Setup ZeroMQ
        logging.info("Initializing ZeroMQ...")
        ctxt = zmq.Context()
        self.set_pnt_sub = ctxt.socket(zmq.SUB)
        self.set_pnt_sub.bind(ZMQ_CONTROL_SETPOINTS)
        self.set_pnt_sub.setsockopt_string(zmq.SUBSCRIBE, '')

        self.graph_data = ctxt.socket(zmq.PUB)
        self.graph_data.connect(ZMQ_GRAPH_DATA_TOPIC)

        self.set_pnt_return = ctxt.socket(zmq.PUB)
        self.set_pnt_return.connect(ZMQ_CURRENT_SET_CONTROLS)

        self.measurement_data = ctxt.socket(zmq.PUB)
        self.measurement_data.connect(ZMQ_MEASURED_VALUES)

        self.poller = zmq.Poller()
        self.poller.register(self.set_pnt_sub, zmq.POLLIN)
        logging.info("ZeroMQ finished init...")

        # Setup motor control
        logging.info("Setting up motor control...")
        self.motor = Roboclaw(self.ROBOCLAW_COMPORT, self.ROBOCLAW_BAUDRATE)
        r = self.motor.Open()
        if r < 1:
            logging.info("Error: could not connect to Roboclaw...")
            logging.info("Exit...")
            exit(0) # TODO Should this be more graceful? 
        logging.info("Motor thread started successfully...")

        #Setup i2C bus
        logging.info("Setting up sensors...")
        logging.info("**Pressure Sensor...")
        logging.info('  Done')

        logging.info("**Flow Sensor...")
        self.bus = SMBus(1) #create I2C bus
        #Setup flow sensor
        self.bus.write_byte_data(self.FLOW_SENSOR_ADDRESS, 0x0B, 0x00) #initialize the I2C device
        logging.info('  Done')

        logging.info('**Oxygen Sensor...')
        self.bus_ADC = busio.I2C(board.SCL, board.SDA)

        #Setup oxygen sensor
        # Create the I2C bus
        #i2c = busio.I2C(board.SCL, board.SDA)

        # Create the ADC object using the I2C bus
        self.ads = ADS.ADS1115(self.bus_ADC)
        # you can specify an I2C adress instead of the default 0x48
        # ads = ADS.ADS1115(i2c, address=0x49)

        # Create single-ended input on channel 3
        self.chan = AnalogIn(self.ads, ADS.P3)
        logging.info('  Done')

        logging.info('**Endstops')
        # Setup End Stop
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.END_STOP_TOP, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Usually High (True), Low (False) when triggered
        GPIO.setup(self.END_STOP_BOTTOM, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Usually High (True), Low (False) when triggered
        GPIO.add_event_detect(self.END_STOP_TOP, GPIO.RISING, callback=self.topEndstop.set, bouncetime=200) 
        GPIO.add_event_detect(self.END_STOP_BOTTOM, GPIO.RISING, callback=self.bottomEndstop.set, bouncetime=200) 
        logging.info('  Done')
        
        self.zeroMotor(self.DIR_UP)

        # # Initial states for state machine
        self.state = State.STOPPED
        self.prev_state = State.STOPPED
        
        #Initialize guisetpoint
        self.acting_guisetpoint =  [500,15,0.5,True]
        self.new_guisetpoint =  [500,15,0.5,True]

    '''
    def handleTopEndstop(self):
        pass

    def handleBottomEndstop(self):
        pass
    '''

    def calcBreathTimePartition(self, inspiration_period, bpm):
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

    def calcVolume(self):
        #NOTE -- may need to add a signal delay before calling this in the loop
        # Start the sensor - [D040] <= 0x06
        self.bus.write_i2c_block_data(self.FLOW_SENSOR_ADDRESS, 0x00, self.START_BITS)
        
        # Tell the sensor we want to read the flow value [D051/D052] => Read Compensated Flow value
        self.bus.write_i2c_block_data(self.FLOW_SENSOR_ADDRESS, 0x00, self.FLOW_BITS)

        # Read the values
        r = self.bus.read_i2c_block_data(self.FLOW_SENSOR_ADDRESS, 0x07, 2)
        i = int.from_bytes(r, byteorder='big') # NOTE: try 'big' if not working

        # Do the conversion
        rd_flow = ((i -  1024.0) * self.FLOW_SENSOR_RANGE / 60000.0) # convert to [L/min](x10)
        return rd_flow

    def calcPressure(self):
        #NOTE -- may need to add a signal delay before calling this in the loop
        # Tell the sensor we want to read the flow value [D051/D052] => Read Compensated Flow value
        answer = self.bus.read_word_data(self.PRESSURE_SENSOR_ADDRESS, self.INIT_SIGNAL)
        
        #bit  shift to get the full value
        answer=float((((answer&0x00FF)<< 8) + ((answer&0xFF00) >> 8)))
        pressure = (answer-self.SENSOR_COUNT_MIN)*(self.SENSOR_PRESSURE_MAX- self.SENSOR_PRESSURE_MIN)/(self.SENSOR_COUNT_MAX-self.SENSOR_COUNT_MIN) + self.SENSOR_PRESSURE_MIN
        return round(pressure,2)

    def calcOxygen(self):
        return self.chan.voltage

    def zeroMotor(self, direction=-1):
        #intialize motor setpoint to 0
        logging.info("Initiializing 0 location...")
        self.motor.SetEncM1(self.ROBOCLAW_ADDRESS,-self.MAX_ENCODER_COUNT) #set current loaction to maximum pull possible
        while not self.hallEffectEvent.is_set():
            self.motor.SpeedM1(self.ROBOCLAW_ADDRESS, direction*100)
        self.motor.SpeedM1(self.ROBOCLAW_ADDRESS, 0)
        self.motor.ResetEncoders(self.ROBOCLAW_ADDRESS)
        self.hallEffectEvent.clear()

    def run(self):
        state_entry_time = time.time()
        logging.info(f"Entering state machine with state {self.acting_guisetpoint}...")

        # Run a state machine to create the waveform output we want
        while not self.quitEvent.is_set():

            # Poll the subscriber for new setpoints
            socks = dict(self.poller.poll(self.ZMQ_POLL_SUBSCRIBER_TIMEOUT_MS))
            if self.set_pnt_sub in socks:
                self.new_guisetpoint = self.set_pnt_sub.recv_pyobj()
                vol, bpm, ie, stopped = self.new_guisetpoint
                t_recent_heartbeat = time.time() # TODO not used yet - can use to check how old last command is...
                self.set_pnt_return.send_pyobj((vol, ie, bpm, stopped))
                print(f"Recieved a new setpoint of {self.new_guisetpoint}" + " "*20)

            # Unpack the setpoints
            vol, bpm, ie, stopped = self.acting_guisetpoint
            
            # Check if the endstops have been hit...
            if self.bottomEndstop.is_set() or self.topEndstop.is_set():
                # Stop the motor and go to stop state
                self.motor.SpeedM1(self.ROBOCLAW_ADDRESS, 0)
                self.state = State.STOPPED
                self.new_guisetpoint[3] = True
                self.acting_guisetpoint[3] = True

            # Check if stop requested
            if stopped:
                self.state = State.STOPPED

            # Calculate breathing period
            Tbreath = 1 / bpm * 60
            self.pressure_list.setPeriod(Tbreath)

            # Calculate inspiration time
            Tinsp = Tbreath * ie
            self.volume_list.setPeriod(Tinsp)

            # Calculate the time partition for the non insp states
            Tnoninsp = self.calcBreathTimePartition(Tinsp, bpm)

            # Read all the sensors...
            #Calculate flow from device
            flow = self.calcVolume()
            #Calculate pressure from device
            pressure = self.calcPressure()       
            # Calculate O2% from device
            oxygen = self.calcOxygen()
            
            # Communicate data to the GUI
            sensor_readings = (pressure, flow, oxygen)
            # Add values to pressure list
            append_time = time.time()
            self.pressure_list.append(pressure, append_time)
            self.pressure_list.update(append_time)
            self.volume_list.append(flow, append_time)
            self.volume_list.update(append_time)
            
            # Build and send message from current values
            self.graph_data.send_pyobj((time.time(), flow, pressure))
            self.measurement_data.send_pyobj((oxygen, self.pressure_list.getMin(), self.pressure_list.getMax()))
                
            # Calculate time we have been in this state so far
            t = time.time() - state_entry_time

            # STATE MACHINE LOGIC
            if self.state == State.INSPR:
                s = f"In state {self.STATES[self.state]} for {t:3.2f}/{Tinsp} s | sensor readings {sensor_readings}" + " "*20
                logging.info(s, end='\r')
                # breath in
                if t > Tinsp:
                    # TODO Stop motor movement
                    # Change states to hold
                    self.state = State.HOLD
                    self.prev_state = State.INSPR
                    state_entry_time = time.time()
                    self.acting_guisetpoint = self.new_guisetpoint
                    self.tidal_volume = self.volume_list.integrate()
                else:
                    # Calc and apply motor rate to zero
                    # Get the slope
                    slope = vol / Tinsp
                    slope_encoder = int(slope / self.K_VOL_TO_ENCODER_COUNT)
                    accel_encoder = int(slope_encoder * self.ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS)
                    self.motor.SpeedAccelDeccelPositionM1(
                        self.ROBOCLAW_ADDRESS, accel_encoder, slope_encoder, accel_encoder, 0, 0)
            elif self.state == State.HOLD:
                # hold current value
                s = f"In state {self.STATES[self.state]} for {t:3.2f}/{Tnoninsp} s | sensor readings {sensor_readings}" + " "*20
                logging.info(s, end='\r')
                if t > Tnoninsp:
                    if self.prev_state == State.INSPR:
                        self.state = State.OUT
                        self.prev_state = State.HOLD
                    elif self.prev_state == State.OUT:
                        self.state = State.INSPR
                        self.prev_state = State.OUT
                    state_entry_time = time.time()
            elif self.state == State.OUT:
                s = f"In state {self.STATES[self.state]} for {t:3.2f}/{Tnoninsp} s | sensor readings {sensor_readings}" + " "*20
                logging.info(s, end='\r')
                if t > Tnoninsp:
                    self.state = State.HOLD
                    self.prev_state = State.OUT
                    state_entry_time = time.time()
                else:
                    # Calc count position of the motor from vol
                    counts = int(vol/self.K_VOL_TO_ENCODER_COUNT)
                    speed_counts = int(counts / Tnoninsp * 1.1)  # A little faster for wiggle room
                    accel_counts = int(speed_counts * self.ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS)
                    self.motor.SpeedAccelDeccelPositionM1(
                        self.ROBOCLAW_ADDRESS, accel_counts, speed_counts, accel_counts, -counts, 0)
            elif self.state == State.STOPPED:
                # Set speed of motor to 0
                self.motor.ForwardM1(self.ROBOCLAW_ADDRESS, 0)
                logging.info(f"In state {self.STATES[self.state]}, waiting for start signal from GUI... | sensor readings {sensor_readings}" + " "*20, end='\r')
                self.acting_guisetpoint = self.new_guisetpoint

                if stopped == False:
                    logging.info(f"Recieved start signal with setpoint {self.acting_guisetpoint}")
                    self.zeroMotor()
                    # Initial states for state machine
                    self.state = State.OUT
                    self.prev_state = State.HOLD
                    state_entry_time = time.time()

                    logging.info("Beggining to breath...")
            elif self.state == State.ERROR:
                # inform something went wrong
                logging.info("There was an error. Exiting...")
                self.quitEvent.set()

        logging.info("Exit state machine. Program done.")

if __name__ == '__main__':

    osvc = OSVController()

    def sigint_handle(signal_recieved, frame, osv: OSVController):
        """Handles a CNTL+C from the user. Should exit gracefully.
        """
        # TODO Need to kill the motor here and clean up
        osv.quitEvent.set()
        print("\nProgram interupted. Exiting...")

    # Setup SIGINT handler
    signal(SIGINT, lambda s, f: sigint_handle(s, f, osvc))
    # Run thread
    osvc.start()
    # Wait to join
    osvc.join()
    exit(0)

