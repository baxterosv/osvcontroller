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


class OperationMode(Enum):
    VOLUME_CONTROL = 0
    PRESSURE_CONTROL = 1
    PRESSURE_SUPPORTED_CONTROL = 2


class Alarm():
    
    def __init__(self, name="", warning_text="", ub=1.0, lb=-1.0, suppress_period=30.0, severity=1):
        self.name = name
        self.warning_text = warning_text
        self.ub = ub
        self.lb = lb
        self.suppress_period = suppress_period
        self.enabled = False
        self.triggered = False
        self.suppressed_mark = 0
        self.severity = severity

    def enable(self):
        self.enabled = True
    
    def disable(self):
        self.enabled = False
    
    def check(self, value):
        if self.enabled and time.time() - self.suppressed_mark > self.suppress_period:
            if value > self.ub or value < self.lb:
                self.triggered = True
            else:
                self.triggered = False
    
    def suppress(self):
        self.triggered = False
        self.suppressed_mark = time.time()
    
    def isTriggered(self, parameter_list):
        return self.triggered

    def getSeverity(self):
        return self.severity
    
    def getWarningText(self):
        return self.warning_text

'''
class Alarm():
    def __init__(self, name, statusMessage, suppressionLength):
        self.name = name
        self.enabled = False
        self.statusMessage = statusMessage
        self.setPoint = None
        self.triggeredPoint = None
        self.alarmState = AlarmState.DISABLED
        self.suppressionLength = suppressionLength
        self.timeSuppressed = 0

    def suppressed(self):
        if self.alarmState != AlarmState.SUPPRESSED:
            return False
        else:
            if (time.time() - self.timeSuppressed) >= self.suppressionLength:
                self.alarmState = AlarmState.TRIGGERED
                self.timeSuppressed = 0
                return False
            else:
                return True

    def enabled(self):
        return enabled

    def enable(self, setPoint=None):
        self.enabled = True
        self.setPoint = setPoint
        self.alarmState = AlarmState.NONE

    def disable(self):
        self.enabled = False
        self.alarmState = AlarmState.DISABLED

    def setState(self, state, value=None):
        if self.enabled():
            if state == AlarmState.TRIGGERED:
                if not self.suppressed():
                    self.alarmState = AlarmState.TRIGGERED
                    self.triggeredPoint = value
            elif state == AlarmState.SUPPRESSED:
                if not self.suppressed():
                    self.alarmState == AlarmState.SUPPRESSED
                    self.timeSuppressed == time.time()
            else:
                self.alarmState = state
                self.timeSuppressed = 0
                self.triggeredPoint = None

    def getState(self):
        if self.enabled():
            self.suppressed()
        return alarmState

    def setSetPoint(self, setPoint):
        self.setPoint = setPoint

    def getSetPoint(self):
        return self.setPoint

    def getTriggeredPoint(self):
        return triggeredPoint

    def setStatusMessage(self, message):
        self.statusMessage = message

    def getStatusMessage(self):
        return self.statusMessage
'''

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
        self.STATES = {State.ERROR: "ERROR", State.INSPR: "INSPIRATION",
                       State.HOLD: "HOLDING", State.OUT: "BREATH_OUT", State.STOPPED: "STOPPED"}

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
        self.MAX_ENCODER_COUNT = 6500  # max number of counts

        self.HALL_EFFECT_SENSOR = 24  # Hall-effect sensor
        self.END_STOP_MARGIN = 50  # margin to move back after hitting the endstop

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

        #Omron Flow Sensor settings
        self.FLOW_SENSOR_ADDRESS_OMRON_OMRON = 0x6C             # Try 0x6F if this doesn't work
        self.FLOW_OMRON_RANGE = 50.0                            # From datasheet
        self.FLOW_OMRON_START_BITS = [0xD0, 0x40, 0x18, 0x06]   # From datasheet, bits to write to start sensor
        self.FLOW_OMRON_MEASURE_BITS = [0xD0, 0x51, 0x2C, 0x07] # From datasheet, bits to read flow

        #Honeywell Pressure sensor settings
        self.PRESSURE_SENSOR_ADDRESS_HONEYWELL = 0x28   # Found using i2cdetect -y 1 on Raspberry Pi
        self.PRESSURE_HONEYWELL_COUNT_MIN = 1638.3      # From datasheet, starting at 10% on bottom
        self.PRESSURE_HONEYWELL_COUNT_MAX = 14744.7     # From datasheet, ending at 90% on top
        self.PRESSURE_HONEYWELL_PRESSURE_MIN = -1.0     # From datasheet, psi
        self.PRESSURE_HONEYWELL_PRESSURE_MAX = 1.0      # From datasheet, psi
        self.PRESSURE_HONEYWELL_INIT_SIGNAL = 0x01      # Init signal for i2c
        self.PSI_2_CMH2O = 70.307                       # Conversion factor

        #Allsensor Pressure sensor settings
        self.PRESSURE_SENSOR_ADDRESS_ALLSENSOR = 0x29   # Found using i2cdetect -y 1 on Raspberry Pi
        self.PRESSURE_ALLSENSOR_OFFSET = 0.5*2**24      # From datasheet, value from table for L30D
        self.PRESSURE_ALLSENSOR_FULLSCALE = 2 * 29.92   # From datasheet, 2x as diff, range 29.92 inH2O (1.08 psi)
        self.PRESSURE_ALLSENSOR_START_SINGLE = 0xAA     # Signal for i2c read
        self.INH2O_2_CMH2O = 2.54                       # Conversion factor


        # Status Colors
        self.RED = (255, 0, 0)

        self.quitEvent = Event()
        self.hallEffectEvent = Event()

        self.pressure_list = TimeManagedList()
        self.volume_list = TimeManagedList()

        self.tidal_volume = None

        # Setup ZeroMQ
        logging.info("Initializing ZeroMQ...")
        ctxt = zmq.Context()

        self.control_settings_sub = ctxt.socket(zmq.SUB)
        self.control_settings_sub.bind(ZMQ_CONTROLLER_SETTINGS)
        self.control_settings_sub.setsockopt_string(zmq.SUBSCRIBE, '')

        self.mute_alarms_sub = ctxt.socket(zmq.SUB)
        self.mute_alarms_sub.bind(ZMQ_MUTE_ALARMS)
        self.mute_alarms_sub.setsockopt_string(zmq.SUBSCRIBE, '')

        self.control_settings_echo_pub = ctxt.socket(zmq.PUB)
        self.control_settings_echo_pub.connect(ZMQ_CONTROLLER_SETTINGS_ECHO)

        self.graph_data = ctxt.socket(zmq.PUB)
        self.graph_data.connect(ZMQ_GRAPH_DATA_TOPIC)

        self.triggered_alarms_pub = ctxt.socket(zmq.PUB)
        self.triggered_alarms_pub.connect(ZMQ_TRIGGERED_ALARMS)

        self.status_pub = ctxt.socket(zmq.PUB)
        self.status_pub.connect(ZMQ_OSV_STATUS)

        self.measurement_data = ctxt.socket(zmq.PUB)
        self.measurement_data.connect(ZMQ_MEASURED_VALUES)

        self.subscribers_poller = zmq.Poller()
        self.subscribers_poller.register(self.control_settings_sub, zmq.POLLIN)
        self.subscribers_poller.register(self.mute_alarms_sub, zmq.POLLIN)

        logging.info("ZeroMQ finished init...")

        # Setup motor control
        logging.info("Setting up motor control...")
        self.motor = Roboclaw(self.ROBOCLAW_COMPORT, self.ROBOCLAW_BAUDRATE)
        r = self.motor.Open()
        if r < 1:
            logging.info("Error: could not connect to Roboclaw...")
            logging.info("Exit...")
            exit(0)  # TODO Should this be more graceful?
        logging.info("Motor thread started successfully...")

        # Setup i2C bus
        logging.info("Setting up sensors...")
        logging.info("**Pressure Sensor...")
        logging.info('  Done')

        logging.info("**Flow Sensor...")
        self.bus = SMBus(1) #create I2C bus
        # Setup flow sensor
        # initialize the I2C device
        self.bus.write_byte_data(self.FLOW_SENSOR_ADDRESS_OMRON, 0x0B, 0x00) #initialize the I2C device

        logging.info('  Done')
        '''
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
        '''
        logging.info('**Endstops')
        # Setup End Stop
        GPIO.setmode(GPIO.BCM)
        # Usually High (True), Low (False) when triggered
        GPIO.setup(self.HALL_EFFECT_SENSOR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.HALL_EFFECT_SENSOR, GPIO.RISING,
                              callback=self.hallEffectEvent.set, bouncetime=200)
        logging.info('  Done')

        self.zeroMotor(self.DIR_UP)

        # # Initial states for state machine
        self.state = State.STOPPED
        self.prev_state = State.STOPPED

        # Initialize guisetpoint
        self.acting_guisetpoint = (True, OperationMode.VOLUME_CONTROL, 500, 0.5, 15, 0.5, 5, 10)
        self.new_guisetpoint = (True, OperationMode.VOLUME_CONTROL, 500, 0.5, 15, 0.5, 5, 10)

        self.state_entry_time = 0

        self.opmode_dict = {OperationMode.VOLUME_CONTROL: 'Volume Control', OperationMode.PRESSURE_CONTROL: 'Pressure Control', OperationMode.PRESSURE_SUPPORTED_CONTROL: 'Assisted Breathing'}
        
        # Alarm Setup
        self.SUPPRESSION_LENGTH = 30 # seconds

        self.alarms = {}
        self.alarms["O2"] = Alarm(name="O2", warning_text="FiO2", ub=1.0, lb=0.0, severity=100)
        #self.alarms["PPlat"] = Alarm(name="PPlat", warning_text="Plateau Pressure")
        self.alarms["PIP"] = Alarm(name="PIP", warning_text="PIP", ub=100, lb=0.0, severity=80)
        self.alarms["Peep"] = Alarm(name="Peep", warning_text="Peep", ub=20, lb=0.0, severity=70)
        '''
        self.alarms["TV"] = Alarm(
            "TV", "Tidal Volume", self.SUPPRESSION_LENGTH)
        self.alarms["Breath"] = Alarm(
            "Breath", "Not Breating", self.SUPPRESSION_LENGTH)
        self.alarms["UPS"] = Alarm(
            "UPS", "Power Supply Disconnected", self.SUPPRESSION_LENGTH)
        self.alarms["O2Disconn"] = Alarm(
            "O2Disconn", "O2 Sypply Disconnected", self.SUPPRESSION_LENGTH)
        '''

        self.alarms["O2"].enable()
        self.alarms["PIP"].enable()
        self.alarms["Peep"].enable()

    def endStop_handler(self, motor):
        logging.info("End Stop Handler\n")
        motor.ResetEncoders(self.ROBOCLAW_ADDRESS)
        motor.SpeedAccelDeccelPositionM1(
            self.ROBOCLAW_ADDRESS, 500, 500, 500, -self.END_STOP_MARGIN, 0)
        time.sleep(0.5)
        motor.ResetEncoders(self.ROBOCLAW_ADDRESS)

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
        # NOTE -- may need to add a signal delay before calling this in the loop
        # Start the sensor - [D040] <= 0x06
        self.bus.write_i2c_block_data(self.FLOW_SENSOR_ADDRESS_OMRON, 0x00, self.FLOW_OMRON_START_BITS)
        
        # Tell the sensor we want to read the flow value [D051/D052] => Read Compensated Flow value
        self.bus.write_i2c_block_data(self.FLOW_SENSOR_ADDRESS_OMRON, 0x00, self.FLOW_OMRON_MEASURE_BITS)

        # Read the values
        r = self.bus.read_i2c_block_data(self.FLOW_SENSOR_ADDRESS_OMRON, 0x07, 2)
        i = int.from_bytes(r, byteorder='big') # NOTE: try 'big' if not working

        # Do the conversion
        rd_flow = ((i -  1024.0) * self.FLOW_OMRON_RANGE / 60000.0) 

        return rd_flow # L/min


    def calcPressure_Honeywell(self):
        #NOTE -- may need to add a signal delay before calling this in the loop
        # Tell the sensor we want to read the flow value [D051/D052] => Read Compensated Flow value
        answer = self.bus.read_word_data(self.PRESSURE_SENSOR_ADDRESS_HONEYWELL, self.PRESSURE_HONEYWELL_INIT_SIGNAL)
        
        #bit  shift to get the full value
        answer=float((((answer&0x00FF)<< 8) + ((answer&0xFF00) >> 8)))
        pressure = (answer-self.PRESSURE_HONEYWELL_COUNT_MIN)*(self.PRESSURE_HONEYWELL_PRESSURE_MAX- self.PRESSURE_HONEYWELL_PRESSURE_MIN)/(self.PRESSURE_HONEYWELL_COUNT_MAX-self.PRESSURE_HONEYWELL_COUNT_MIN) + self.PRESSURE_HONEYWELL_PRESSURE_MIN
        
        pressure = presssure * self.PSI_2_CMH2O 

        return round(pressure,2) # cmH2O

    def calcPressure_Allsensor(self):

        # Read data from sensor, 7 bytes long
        reading = bytearray(7)
        self.bus.read_block_data(self.PRESSURE_SENSOR_ADDRESS_ALLSENSOR, self.PRESSURE_ALLSENSOR_START_SINGLE, reading)
        # Pressure data is in proper order in bytes 2, 3 and 4
        reading = (reading&0x00FFFFFF000000)>>24


        pressure = 1.25 * ((reading - self.PRESSURE_ALLSENSOR_OFFSET)>>24) * \
                    self.PRESSURE_ALLSENSOR_FULLSCALE * INH2O_2_CMH2O

        return round(pressure,2) # cmH2O


    def calcOxygen(self):
        # return self.chan.voltage
        return 0.5

    def zeroMotor(self, direction=-1):
        # intialize motor setpoint to 0
        logging.info("Initiializing 0 location...")
        # set current loaction to maximum pull possible
        self.motor.SetEncM1(self.ROBOCLAW_ADDRESS, -self.MAX_ENCODER_COUNT)
        while not self.hallEffectEvent.is_set():
            self.motor.SpeedM1(self.ROBOCLAW_ADDRESS, direction*100)
        self.motor.SpeedM1(self.ROBOCLAW_ADDRESS, 0)
        self.motor.ResetEncoders(self.ROBOCLAW_ADDRESS)
        self.hallEffectEvent.clear()

    def getAlarms(self):
        l = list()
        for a in list(self.alarms.values()):
            if a.isTriggered():
                l.append(a)
        if len(l) > 0:
            m = max(l, key=lambda p: p.getSeverity())
            return m.getWarningText()
        
        return None
        
    def volume_control_state_machine(self):
        stopped, _, vtv, vie, vrr, _, _, _ = self.acting_guisetpoint
        
        if stopped:
            self.state = State.STOPPED

        # Calculate time we have been in this state so far
        t = time.time() - self.state_entry_time

        # Calculate breathing period
        Tbreath = 1 / vrr * 60
        self.pressure_list.setPeriod(Tbreath)

        # Calculate inspiration time
        Tinsp = Tbreath * vie
        self.volume_list.setPeriod(Tinsp)

        # Calculate the time partition for the non insp states
        Tnoninsp = self.calcBreathTimePartition(Tinsp, vrr)

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
                self.state_entry_time = time.time()
                self.acting_guisetpoint = self.new_guisetpoint
                self.tidal_volume = self.volume_list.integrate()
            else:
                # Calc and apply motor rate to zero
                # Get the slope
                slope = vtv / Tinsp
                slope_encoder = int(slope / self.K_VOL_TO_ENCODER_COUNT)
                accel_encoder = int(
                    slope_encoder * self.ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS)
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
                self.state_entry_time = time.time()
        elif self.state == State.OUT:
            s = f"In state {self.STATES[self.state]} for {t:3.2f}/{Tnoninsp} s | sensor readings {sensor_readings}" + " "*20
            logging.info(s, end='\r')
            if t > Tnoninsp:
                self.state = State.HOLD
                self.prev_state = State.OUT
                self.state_entry_time = time.time()
            else:
                # Calc count position of the motor from vol
                counts = int(vtv/self.K_VOL_TO_ENCODER_COUNT)
                # A little faster for wiggle room
                speed_counts = int(counts / Tnoninsp * 1.1)
                accel_counts = int(
                    speed_counts * self.ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS)
                self.motor.SpeedAccelDeccelPositionM1(
                    self.ROBOCLAW_ADDRESS, accel_counts, speed_counts, accel_counts, -counts, 0)
        elif self.state == State.STOPPED:
            # Set speed of motor to 0
            self.motor.ForwardM1(self.ROBOCLAW_ADDRESS, 0)
            logging.info(
                f"In state {self.STATES[self.state]}, waiting for start signal from GUI... | sensor readings {sensor_readings}" + " "*20, end='\r')
            self.acting_guisetpoint = self.new_guisetpoint

            if stopped == False:
                logging.info(
                    f"Recieved start signal with setpoint {self.acting_guisetpoint}")
                self.zeroMotor()
                # Initial states for state machine
                self.state = State.OUT
                self.prev_state = State.HOLD
                self.state_entry_time = time.time()
                logging.info("Beggining to breath...")
        elif self.state == State.ERROR:
            # inform something went wrong
            logging.info("There was an error. Exiting...")
            self.quitEvent.set()

    def pressure_control_state_machine(self):
        stopped, _, _, vie, vrr, _, _, _ = self.acting_guisetpoint
        vtv = 700
        if stopped:
            self.state = State.STOPPED

        # Calculate time we have been in this state so far
        t = time.time() - self.state_entry_time

        # Calculate breathing period
        Tbreath = 1 / vrr * 60
        self.pressure_list.setPeriod(Tbreath)

        # Calculate inspiration time
        Tinsp = Tbreath * vie
        self.volume_list.setPeriod(Tinsp)

        # Calculate the time partition for the non insp states
        Tnoninsp = self.calcBreathTimePartition(Tinsp, vrr)

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
                self.state_entry_time = time.time()
                self.acting_guisetpoint = self.new_guisetpoint
                self.tidal_volume = self.volume_list.integrate()
            else:
                # Calc and apply motor rate to zero
                # Get the slope
                slope = vtv / Tinsp
                slope_encoder = int(slope / self.K_VOL_TO_ENCODER_COUNT)
                accel_encoder = int(
                    slope_encoder * self.ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS)
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
                self.state_entry_time = time.time()
        elif self.state == State.OUT:
            s = f"In state {self.STATES[self.state]} for {t:3.2f}/{Tnoninsp} s | sensor readings {sensor_readings}" + " "*20
            logging.info(s, end='\r')
            if t > Tnoninsp:
                self.state = State.HOLD
                self.prev_state = State.OUT
                self.state_entry_time = time.time()
            else:
                # Calc count position of the motor from vol
                counts = int(vtv/self.K_VOL_TO_ENCODER_COUNT)
                # A little faster for wiggle room
                speed_counts = int(counts / Tnoninsp * 1.1)
                accel_counts = int(
                    speed_counts * self.ROBOCLAW_CONTROL_ACCEL_AGGRESSIVENESS)
                self.motor.SpeedAccelDeccelPositionM1(
                    self.ROBOCLAW_ADDRESS, accel_counts, speed_counts, accel_counts, -counts, 0)
        elif self.state == State.STOPPED:
            # Set speed of motor to 0
            self.motor.ForwardM1(self.ROBOCLAW_ADDRESS, 0)
            logging.info(
                f"In state {self.STATES[self.state]}, waiting for start signal from GUI... | sensor readings {sensor_readings}" + " "*20, end='\r')
            self.acting_guisetpoint = self.new_guisetpoint

            if stopped == False:
                logging.info(
                    f"Recieved start signal with setpoint {self.acting_guisetpoint}")
                self.zeroMotor()
                # Initial states for state machine
                self.state = State.OUT
                self.prev_state = State.HOLD
                self.state_entry_time = time.time()
                logging.info("Beggining to breath...")
        elif self.state == State.ERROR:
            # inform something went wrong
            logging.info("There was an error. Exiting...")
            self.quitEvent.set()

    def assisted_breathing_state_machine(self):
        raise NotImplementedError


    def run(self):
        self.state_entry_time = time.time()
        logging.info(
            f"Entering state machine with state {self.acting_guisetpoint}...")

        # Run a state machine to create the waveform output we want
        while not self.quitEvent.is_set():

            # Poll the subscriber for new setpoints
            socks = dict(self.subscribers_poller.poll(
                self.ZMQ_POLL_SUBSCRIBER_TIMEOUT_MS))

            if self.control_settings_sub in socks:
                self.new_guisetpoint = self.control_settings_sub.recv_pyobj()
                self.control_settings_echo_pub.send_pyobj(self.new_guisetpoint)
                s = f"Recieved a new setpoint of {self.new_guisetpoint}" + " "*20
                logging.info(s)
            if self.mute_alarms_sub in socks:
                b = self.mute_alarms_sub.recv_pyobj()
                if b:
                    for a in list(self.alarms.values()):
                        if a.isTriggered():
                            a.suppress()

            # Unpack the setpoints
            _, opmode, _, _, _, vdo2, vpeep, vpp = self.acting_guisetpoint

            # Read all the sensors...
            # Calculate flow from device
            flow = self.calcVolume()

            #Calculate pressure from device
            pressure = self.calcPressure_Allsensor()       

            # Calculate O2% from device
            oxygen = self.calcOxygen()

            # Add values to pressure list
            append_time = time.time()
            self.pressure_list.append(pressure, append_time)
            self.pressure_list.update(append_time)
            self.volume_list.append(flow, append_time)
            self.volume_list.update(append_time)

            # Build and send message from current values
            self.graph_data.send_pyobj((time.time(), flow, pressure))
            self.measurement_data.send_pyobj(
                (oxygen, self.pressure_list.getMin(), self.pressure_list.getMax()))

            self.alarms["O2"].check(vdo2)
            self.alarms["Peep"].check(vpeep)
            self.alarms["PIP"].check(vpp)

            warning_text = self.getAlarms()

            if warning_text != None:
                self.status_pub.send_pyobj(f'Alarm -> {warning_text}')
                self.triggered_alarms_pub.send_pyobj(True)
            else:
                s = self.opmode_dict[opmode]
                self.status_pub.send_pyobj('Nominal in {s} mode')
                self.triggered_alarms_pub.send_pyobj(False)

            if opmode == OperationMode.VOLUME_CONTROL:
                self.volume_control_state_machine()
            elif opmode == OperationMode.PRESSURE_CONTROL:
                self.pressure_control_state_machine()
            elif opmode == OperationMode.PRESSURE_SUPPORTED_CONTROL:
                self.assisted_breathing_state_machine()

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
