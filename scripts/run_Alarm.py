import RPi.GPIO as GPIO
import time


ALARM_PIN = 5

GPIO.setmode(GPIO.BCM)
# Usually High (True), Low (False) when triggered
GPIO.setup(ALARM_PIN, GPIO.OUT, initial=GPIO.LOW)


print ("Alarm in 5 seconds")

time.sleep(5)

GPIO.output(ALARM_PIN,GPIO.HIGH)

print ("Alarm for 5 seconds")

time.sleep(5)

GPIO.output(ALARM_PIN, GPIO.LOW)

print ("Alarm off")

GPIO.cleanup
