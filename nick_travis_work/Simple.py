import time
import RPi.GPIO as GPIO

#pwm
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
motor = GPIO.PWM(13, 50) #pin, frequency
end_stop = 18
GPIO.setup(end_stop,GPIO.IN,pull_up_down=GPIO.PUD_UP)

PIN_HALL = 22
GPIO.setup(PIN_HALL,GPIO.IN)

#start motor
motor.start(0)
#while not GPIO.input(end_stop):
#    motor.ChangeDutyCycle(6.4)
#    continue
#while GPIO.input(end_stop):
#    motor.ChangeDutyCycle(7.6)
#    time.sleep(0.01)
#    continue
#motor.ChangeDutyCycle(7)
print("Initializing")

while GPIO.input(PIN_HALL):
    motor.ChangeDutyCycle(6.4)

motor.ChangeDutyCycle(7)

print("Motor Started...")
time.sleep(1)
t = time.time()
try:
    while (time.time() - t) < 8:
        if((time.time() - t) < 1):
            motor.ChangeDutyCycle(7.8)
            print("Exhaling...")
        elif((time.time() - t) < 2):
            motor.ChangeDutyCycle(7.0)
            print("Holding...")
        elif((time.time() - t) < 3 or GPIO.input(PIN_HALL)):
            motor.ChangeDutyCycle(6.2)
            print("Inhaling")
        elif((time.time() - t)< 1):
            motor.ChangeDutyCycle(7.0)
            print("Holding...")
except KeyboardInterrupt:
    print("\nYou've exited the program.\n")
    
finally:
    GPIO.cleanup()

motor.stop()
GPIO.cleanup()
