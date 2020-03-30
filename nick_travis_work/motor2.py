import RPi.GPIO as GPIO
import time
import threading


class PID:
    def __init__(self, p, i, d, setpoint, total_vol):
        self.p = p
        self.i = i
        self.d = d
        self.min_num = 0
        self.max_num = 0
        self.setpoint = setpoint  # in ml
        self.total_vol = total_vol

    def getSetpoint(self):
        return self.setpoint

    def setSetpoint(self, setpoint, total_vol):
        self.setpoint = setpoint
        self.total_vol = total_vol
        
    def error(self, encoder_position):
        error = self.setpoint - encoder_position
        #print(f"Error: {error}")
        #if abs(error) < 20:
            #return 0
        return error

    def setMinMax(self, min_num, max_num):
        self.min_num = min_num
        self.max_num = max_num

    def limit(self, num, minn, maxn):  # limit value between a min and a max
        return max(min(maxn, num), minn)

    # only P calculations made, I, and D might not be needed
    def calculate(self, encoder_position):
        output = round(self.p * self.error(encoder_position)/self.total_vol,1)
        return self.limit(output, self.min_num, self.max_num)


class Encoder(threading.Thread):
    def __init__(self, A_pin, B_pin,COUNT_TO_VOL):
        threading.Thread.__init__(self)
        self.A_pin = A_pin
        self.B_pin = B_pin
        self.COUNT_TO_VOL = COUNT_TO_VOL
        self.prev_AB = 0b00
        self.counter = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(A_pin, GPIO.IN)
        GPIO.setup(B_pin, GPIO.IN)
        self._stop = threading.Event()
        self.lock = threading.Lock()

    def run(self):
        while True:
            if self.stopped():
                return
            self.updatePosition()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def updatePosition(self):
        outcome = [0, -1, 1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]
        A = GPIO.input(self.A_pin)
        B = GPIO.input(self.B_pin)
        current_AB = (A << 1) | B
        position = (self.prev_AB << 2) | current_AB
        with self.lock:
            self.counter += outcome[position]
        self.prev_AB = current_AB

    def position(self):
        return self.counter*self.COUNT_TO_VOL

    def resetEncoder(self):
        self.counter = 0


class TalonMotor(threading.Thread):

    def __init__(self, pwm_pin, encoder_pin_A, encoder_pin_B, Kp, Ki, Kd,COUNT_TO_VOL, total_vol,END_STOP, pin_hall):
        threading.Thread.__init__(self)
        self.encoder = Encoder(encoder_pin_A, encoder_pin_B,COUNT_TO_VOL)
        self.pid = PID(Kp, Ki, Kd, 0, total_vol)
        self.pid.setMinMax(-1.0, 1.0)
        self.port = pwm_pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.port, GPIO.OUT)
        self.motor = GPIO.PWM(self.port, 50) #pin, frequency
        self.speed = 0
        self.pwmSpeed = 0
        #set endstop
        self.endStopValue = 0
        self.end_stop = END_STOP
        GPIO.setup(self.end_stop,GPIO.IN,pull_up_down=GPIO.PUD_UP)
        self._stop = threading.Event()
        self.Pin_Hall = pin_hall
        GPIO.setup(self.Pin_Hall,GPIO.IN)
        
        #start motor
        self.motor.start(0)
        while GPIO.input(self.Pin_Hall):
            self.motor.ChangeDutyCycle(6.4)
        
        self.motor.ChangeDutyCycle(7)
        self.encoder.start()

    def run(self):
        while True:
            if self.stopped() or GPIO.input(self.end_stop):
                self.speed = 0
                return
            #with self.encoder.lock:
             #   self.setSpeed(self.pid.calculate(self.encoder.position()))
            self.motor.ChangeDutyCycle(self.pwmSpeed)

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()
    
    def limitHit(self, targetVol):
        hallHit = not GPIO.input(self.Pin_Hall)
        positionReached = False
        if (abs(targetVol - self.encoder.position()) < 30):
            positionReached = true
        return hallHit or positionReached
    
    def setSpeed(self, speed):
        self.speed = speed
        self.pwmSpeed = (self.speed * 3.0) + 7.0

    def setPostion(self, counts):
        """Set the position of the motor in encoder counts. The motor will try to reach this position using the defined PID.
        
        Arguments:
            counts {int} -- number of counts to go to.
        """
        self.pid.setSetpoint(counts)
        
    def setVolume(self, ml, total_vol, max_rate):
        #Set volume of pipe, easier to work with using absolute position
        self.pid.setMinMax(-max_rate, max_rate)
        self.pid.setSetpoint(ml, total_vol) #pass ml seetpouint

