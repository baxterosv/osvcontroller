from smbus import SMBus
from signal import signal, SIGINT
import time

# Allsensor Pressure sensor settings
# Found using i2cdetect -y 1 on Raspberry Pi
PRESSURE_SENSOR_ADDRESS_ALLSENSOR = 0x29
# From datasheet, value from table for L30D
PRESSURE_ALLSENSOR_OFFSET = 0.5*2**24
# From datasheet, 2x as diff, range 29.92 inH2O (1.08 psi)
PRESSURE_ALLSENSOR_FULLSCALE = 2 * 29.92
PRESSURE_ALLSENSOR_START_SINGLE = 0xAA     # Signal for i2c read
INH2O_2_CMH2O = 2.54                       # Conversion factor

SENSOR_MEASUREMENT_WAIT_TIME = 1

# Setup exit gracefully
running = True
def sigint_handle(signal_recieved, frame):
    running = False
signal(SIGINT, sigint_handle)

# Create I2C bus
bus = SMBus(1)

INIT_SIGNAL = 0x01 #init signal fori2c
while(running):

    time.sleep(SENSOR_MEASUREMENT_WAIT_TIME)

    # Read data from sensor, 7 bytes long
    bus.write_byte(PRESSURE_SENSOR_ADDRESS_ALLSENSOR,
                        PRESSURE_ALLSENSOR_START_SINGLE)
    time.sleep(0.01)
    reading = bus.read_i2c_block_data(PRESSURE_SENSOR_ADDRESS_ALLSENSOR, 0, 7)
    # Pressure data is in proper order in bytes 2, 3 and 4
    r = (reading[1] << 16) | (reading[2] << 8) | (reading[3])

    pressure = 1.25 * ((r - PRESSURE_ALLSENSOR_OFFSET)/2**24) * \
        PRESSURE_ALLSENSOR_FULLSCALE * INH2O_2_CMH2O

    print(f"Pressure {pressure:.3f} cm-H2O")


print("Exiting program...")
bus.close()