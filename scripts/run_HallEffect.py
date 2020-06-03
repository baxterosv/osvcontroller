import RPi.GPIO as GPIO
from threading import Event

def endStop_handler(channel):
    print(f"Hall Effect Handler on channel {channel}\n")
    hallEffectEvent.set()


HALL_EFFECT_PIN = 26
hallEffectEvent = Event()

GPIO.setmode(GPIO.BCM)
# Usually High (True), Low (False) when triggered
GPIO.setup(HALL_EFFECT_PIN, GPIO.IN,
           pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(HALL_EFFECT_PIN, GPIO.FALLING,
                      callback=endStop_handler, bouncetime=200)

#print ("Waiting for magnet to trigger hall effect")
#while GPIO.input(HALL_EFFECT_PIN):
while not hallEffectEvent.is_set():
    print("Waiting for magnet to trigger hall effect")
    continue

print ("Set off")
