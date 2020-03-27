import time
from encoderThread import Encoder
e = Encoder(21, 20)
class PID:
	def __init__(self, p, i, d, setpoint):
		self.p = p
		self.i = i
		self.d = d
		self.min_num = 0
		self.max_num = 0
		self.setpoint = setpoint  #in ticks
	def getSetpoint(self):
		return self.setpoint
	def error(self, encoder_position):
		return encoder_position - self.setpoint
	def setMinMax(self, min_num, max_num):
		self.min_num = min_num
		self.max_num = max_num
	def limit(self, num, minn, maxn): #limit value between a min and a max
		return max(min(maxn, num), minn)
	def calculate(self, encoder_position): #only P calculations made, I, and D might not be needed
		output = self.p * self.error(encoder_position)
		return self.limit(output, self.min_num, self.max_num)
#e.start()
#pid = PID(0.001,0,0,1000)
#print(pid.getSetpoint())
#pid.setMinMax(-.75, .75)
#while True:
#	print(pid.calculate(e.position()))
#	time.sleep(2)
