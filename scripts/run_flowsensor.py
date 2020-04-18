from smbus import SMBus
from signal import signal, SIGINT
from time import sleep

FLOW_SENSOR_ADDRESS = 0x6C # Try 0x6F if this doesn't work
SENSOR_MEASUREMENT_WAIT_TIME = 0.120 # s
FLOW_SENSOR_RANGE = 50.0
START_BITS = [0xD0, 0x40, 0x18, 0x06] #from datasheet, bits to write to start sensor
FLOW_BITS = [0xD0, 0x51, 0x2C, 0x07] #from datasheet, bits to read flow
# Setup exit gracefully
running = True
def sigint_handle(signal_recieved, frame):
    running = False
signal(SIGINT, sigint_handle)

# Create I2C bus
bus = SMBus(1)

# Initialize device
bus.write_byte_data(FLOW_SENSOR_ADDRESS, 0x0B, 0x00)


while(running):

    
    # NOTE: May need to move [D040] <= 0x06 inside the loop here, but I don't think you do

    # Start the sensor - [D040] <= 0x06
    bus.write_i2c_block_data(FLOW_SENSOR_ADDRESS, 0x00,START_BITS)

    sleep(SENSOR_MEASUREMENT_WAIT_TIME)

    # Tell the sensor we want to read the flow value [D051/D052] => Read Compensated Flow value
    bus.write_i2c_block_data(FLOW_SENSOR_ADDRESS, 0x00, FLOW_BITS)

    # Read the values
    r = bus.read_i2c_block_data(FLOW_SENSOR_ADDRESS, 0x07, 2)
    i = int.from_bytes(r, byteorder='big') # NOTE: try 'big' if not working

   # Do the conversion
    rd_flow = ((i -  1024.0) * FLOW_SENSOR_RANGE / 60000.0) # convert to [L/min](x10)

    # Output for the user
    print(f"Sensor Reading {rd_flow:.3f} L/min" + " "*20, end='\r')


print("Exiting program...")
bus.close()