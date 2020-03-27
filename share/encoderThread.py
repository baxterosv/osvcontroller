import RPi.GPIO as GPIO
import time
import threading

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
		outcome = [0,-1,1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0]
		A = GPIO.input(self.A_pin)
		B = GPIO.input(self.B_pin)
		current_AB = (A << 1) | B
		position = (self.prev_AB << 2) | current_AB
		self.counter += outcome[position]
		self.prev_AB = current_AB
	def position(self):
		return self.counter
	def resetEncoder(self):
		self.counter = 0

	#print(e.position())
