from smbus import SMBus
from signal import signal, SIGINT
from time import sleep

PRESSURE_SENSOR_ADDRESS = 0x28 # Found using i2cdetect -y 1 on Raspberry Pi
SENSOR_MEASUREMENT_WAIT_TIME = 0.120 # s
SENSOR_COUNT_MIN = 1638.3 #from datasheet, starting at 10% on bottom
SENSOR_COUNT_MAX = 14744.7 #from datasheet, ending at 90% on top
SENSOR_PRESSURE_MIN = -1.0 #from datasheet
SENSOR_PRESSURE_MAX = 1.0 #from datasheet

# Setup exit gracefully
running = True
def sigint_handle(signal_recieved, frame):
    running = False
signal(SIGINT, sigint_handle)

# Create I2C bus
bus = SMBus(1)

INIT_SIGNAL = 0x01 #init signal fori2c
while(running):

    # NOTE: May need to move [D040] <= 0x06 inside the loop here, but I don't think you do

    sleep(SENSOR_MEASUREMENT_WAIT_TIME)

    # Tell the sensor we want to read the flow value [D051/D052] => Read Compensated Flow value
    answer = bus.read_word_data(PRESSURE_SENSOR_ADDRESS,INIT_SIGNAL)
    
    
    #bit  shift to get the full value
    answer=float((((answer&0x00FF)<< 8) + ((answer&0xFF00) >> 8)))




    pressure = (answer-SENSOR_COUNT_MIN)*(SENSOR_PRESSURE_MAX- SENSOR_PRESSURE_MIN)/(SENSOR_COUNT_MAX-SENSOR_COUNT_MIN) + SENSOR_PRESSURE_MIN
    # Output for the user
    print(f"Sensor Reading {pressure:.3f} psi" + " "*20, end='\r')


print("Exiting program...")
bus.close()