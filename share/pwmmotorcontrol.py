import RPi.GPIO as GPIO
import time
import threading
from encoderThread import Encoder
from pid import PID

encoder = Encoder(21, 20) #port nums for encoder
pid = PID(0.001, 0, 0, 2000) # p , i , d , setpoint
pid.setMinMax(-1.0, 1.0)
class PWMMotorControl(threading.Thread):
	def __init__(self, port):
		threading.Thread.__init__(self)
		self.port = port
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.port, GPIO.OUT)
		self.motor = GPIO.PWM(self.port, 50)
		self.speed = 0
		self.pwmSpeed = 0
		self._stop = threading.Event()
		self.controlEnabled = True 
	def run(self):
		self.motor.start(0)
		while self.controlEnabled:
			if self.stopped():
				self.motor.ChangeDutyCycle(0)
				return
			self.motor.ChangeDutyCycle(self.pwmSpeed)
	def stop(self):
		self._stop.set()
	def stopped(self):
		return self._stop.isSet()
	def enableController(self, enable):
		self.controlEnabled = enable
	def setSpeed(self, speed):
		self.speed = speed
		self.pwmSpeed = (self.speed * 3.0) + 7.0
p = PWMMotorControl(19)
p.start()
encoder.start()
while True:
	print(pid.error(encoder.position()))
	p.setSpeed(pid.calculate(encoder.position()))
	if (abs(pid.error(encoder.position())) < 10):
		p.stop()
		encoder.stop()
		break
