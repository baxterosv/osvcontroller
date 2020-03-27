import RPi.GPIO as IO
import time
from encoderThread import Encoder
from pid import PID

encoder = Encoder(21, 20) #port nums for encoder
pid = PID(0.001, 0, 0, 2000) # p , i , d , setpoint
pid.setMinMax(-0.6, 0.6)
IO.setwarnings(False)

IO.setmode(IO.BCM)

IO.setup(19,IO.OUT)
p = IO.PWM(19,50)
p.start(0)

def setPWM(pwm):
	return (pwm - 7)/3 #7 is mid value, 3 is range / 2
def setSpeed(speed):
	return (speed * 3.0) + 7.0
encoder.start()
while True:
	#print(setSpeed(pid.calculate(encoder.position())))
	print(pid.error(encoder.position()))
	p.ChangeDutyCycle(setSpeed(pid.calculate(encoder.position())))
	if (abs(pid.error(encoder.position())) < 30):
		encoder.stop()
		encoder.join()
		break
#	print("working")
#	for x in range(0, 40):
#		p.ChangeDutyCycle(7 - (x/10.0))
#		print(7 - (x/10.0))
#		time.sleep(0.05)
	
