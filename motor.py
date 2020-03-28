import RPi.GPIO as GPIO
import time
import threading


class PID:
    def __init__(self, p, i, d, setpoint):
        self.p = p
        self.i = i
        self.d = d
        self.min_num = 0
        self.max_num = 0
        self.setpoint = setpoint  # in ticks

    def getSetpoint(self):
        return self.setpoint

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint

    def error(self, encoder_position):
        return encoder_position - self.setpoint

    def setMinMax(self, min_num, max_num):
        self.min_num = min_num
        self.max_num = max_num

    def limit(self, num, minn, maxn):  # limit value between a min and a max
        return max(min(maxn, num), minn)

    # only P calculations made, I, and D might not be needed
    def calculate(self, encoder_position):
        output = self.p * self.error(encoder_position)
        return self.limit(output, self.min_num, self.max_num)


class Encoder(threading.Thread):
    def __init__(self, A_pin, B_pin):
        threading.Thread.__init__(self)
        self.A_pin = A_pin
        self.B_pin = B_pin
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
        return self.counter

    def resetEncoder(self):
        self.counter = 0


class TalonMotor(threading.Thread):

    def __init__(self, pwm_pin, encoder_pin_A, encoder_pin_B, Kp, Ki, Kd):
        threading.Thread.__init__(self)
        self.encoder = Encoder(encoder_pin_A, encoder_pin_B)
        self.pid = PID(Kp, Ki, Kd, 0)
        self.pid.setMinMax(-1.0, 1.0)
        self.port = pwm_pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.port, GPIO.OUT)
        self.motor = GPIO.PWM(self.port, 50)
        self.speed = 0
        self.pwmSpeed = 0
        self._stop = threading.Event()

        self.encoder.start()

    def run(self):
        self.motor.start(0)
        while True:
            if self.stopped():
                self.motor.ChangeDutyCycle(0)
                return
            with self.encoder.lock:
                self.setSpeed(self.pid.calculate(self.encoder.position()))
            self.motor.ChangeDutyCycle(self.pwmSpeed)

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def setSpeed(self, speed):
        self.speed = speed
        self.pwmSpeed = (self.speed * 3.0) + 7.0

    def setPostion(self, counts):
        """Set the position of the motor in encoder counts. The motor will try to reach this position using the defined PID.
        
        Arguments:
            counts {int} -- number of counts to go to.
        """
        self.pid.setSetpoint(counts)
