# Import libraries
import RPi.GPIO as GPIO

# Pin selection
PIN_O2_UP = 0       # Increase O2 [%]
PIN_O2_DWN = 2      # Decrease O2 [%]
PIN_TIDE_UP = 12    # Increase tidal volume [mL]
PIN_TIDE_DWN = 13   # Decrease tidal volume [mL]
PIN_RESP_UP = 21    # Increase respiratory rate [b/m]
PIN_RESP_DWN = 22   # Decrease respiratory rate [b/m]
PIN_INHL_UP = 4     # Increase inhalation rate [sec]
PIN_INHL_DWN = 5    # Decrease inhalation rate [sec]
PIN_PEEP_UP= 10     # Increase peep pressure [cm H2O]
PIN_PEEP_DWN = 11   # Decrease peep pressure [cm H2O]


# Set max, min, increment for each variable
O2_MAX = 100        # Max O2 %
O2_MIN = 20         # Min O2 %
O2_INC = 10         # Size of increment in %

TIDE_MAX = 1000     # Max tidal volume in mL
TIDE_MIN = 200      # Min tidal volume in mL
TIDE_INC = 10       # Size of increment in mL

RESP_MAX = 20       # Max tidal volume in b/m
RESP_MIN = 5        # Min tidal volume in b/m
RESP_INC = 1        # Size of increment in b/m

INHL_MAX = 2.0      # Max tidal volume in sec
INHL_MIN = 0.5      # Min tidal volume in sec
INHL_INC = 0.1      # Size of increment in sec
 
PEEP_MAX = 10       # Max tidal volume in cm H2O
PEEP_MIN = 5        # Min tidal volume in cm H2O
PEEP_INC = 1        # Size of increment in cm  H2O


# Global variables to hold current values of each variable
current_O2 = 0
current_Tide = 0
current_Resp = 0
current_Inhl = 0.0
current_Peep = 0


def buttonInitialize()
  global current_O2, current_tide, current_resp, curren_inhl, current_peep

  # Set correct GPIO pins to input
  #####CAN USE INTERNAL PULLUP RESISTORS (IF AVAILABLE) IN GPIO.setup
  GPIO.setup(PIN_02_UP, GPIO.IN)
  GPIO.setup(PIN_02_DWN, GPIO.IN)
  GPIO.setup(PIN_TIDE_UP, GPIO.IN)
  GPIO.setup(PIN_TIDE_DWN, GPIO.IN)
  GPIO.setup(PIN_RESP_UP, GPIO.IN)
  GPIO.setup(PIN_RESP_DWN, GPIO.IN)
  GPIO.setup(PIN_INHL_UP, GPIO.IN)
  GPIO.setup(PIN_INHL_DWN, GPIO.IN)
  GPIO.setup(PIN_PEEP_UP, GPIO.IN)
  GPIO.setup(PIN_PEEP_DWN, GPIO.IN)

  # Add event detection for each pin
  ####BOUNCETIME SHOULD BE TESTED WITH BUTTON, AVOIDS DOUBLE CALL TO HANDLER
  ####CAN DETECT EDGES THAT ARE RISING, FALLING, OR BOTH
  GPIO.add_event_detect(PIN_O2_UP, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)
  GPIO.add_event_detect(PIN_O2_DWN, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)
  GPIO.add_event_detect(PIN_TIDE_UP, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)
  GPIO.add_event_detect(PIN_TIDE_DWN, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)
  GPIO.add_event_detect(PIN_RESP_UP, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)
  GPIO.add_event_detect(PIN_RESP_DWN, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)
  GPIO.add_event_detect(PIN_INHL_UP, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)
  GPIO.add_event_detect(PIN_INHL_DWN, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)
  GPIO.add_event_detect(PIN_PEEP_UP, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)
  GPIO.add_event_detect(PIN_PEEP_DWN, GPIO.FALLING, callback = buttonHandler, bouncetime = 200)

  # Set current values to mid point
  ####WE SHOULD LOOK INTO PROPER DEFAULT VALUES
  Current_O2 = 60
  Current_Tide = 600
  Current_Resp = 12
  Current_Inhl = 1.2
  Current_Peep = 8

# Handles event detect for input buttons
# only make changes that don't put variable out of bounds
####MSG/DO SOMETHING IF TRY TO ADJUST OUT OF RANGE??
# -channel contains GPIO pin number of triggering pin
def buttonHander(channel)
  global current_O2, current_tide, current_resp, curren_inhl, current_peep
  
  if(channel == PIN_O2_UP):
    if(current_O2 < O2_MAX):
      current_O2 += O2_INC

  elif(channel == PIN_O2_DWN):
    if(current_O2 > O2_MIN):
      current_O2 -= O2_INC

  elif(channel == PIN_TIDE_UP):
    if(current_tide < TIDE_MAX):
      current_tide += TIDE_INC

  elif(channel == PIN_TIDE_DWN):
    if(current_tide > TIDE_MIN):
      current_tide -= TIDE_INC

  elif(channel == PIN_RESP_UP):
    if(current_resp < RESP_MAX):
      current_resp += RESP_INC

  elif(channel == PIN_RESP_DWN):
    if(current_resp > RESP_MIN):
      current_resp -= RESP_INC

  elif(channel == PIN_INHL_UP):
    if(current_inhl < INHL_MAX):
      current_inhl += INHL_INC

  elif(channel == PIN_INHL_DWN):
    if(current_inhl > INHL_MIN):
      current_inhl -= INHL_INC

  elif(channel == PIN_PEEP_UP):
    if(current_peep < PEEP_MAX):
      current_peep += PEEP_INC

  elif(channel == PIN_PEEP_DWN):
    if(current_peep > PEEP_MIN):
      current_peep -= PEEP_INC

  else:
    printf("Unknown event triggered")
    ####BETTER ERROR HANDLING


def main():

  # Wrap in try block to exit GPIO cleanly
  try: 
    
    # Tell GPIO library to use GPIO pin names
    GPIO.setmode(GPIO.BCM)

    # Initialize the current values and button handlers
    buttonInitialize

    # Just wait for button events
    ####ANY REAL CODE SHOULD TAKE THIS STATEMENTS PLACE
    while(true):
      continue
   

  # Execute if CTRL + C used to end program
  except KeyboardInterrupt:
    print("\nYou've exited the program")

  ####ADD OTHER EXCEPTIONS AS DESIRED

  # Execute anytime program is exited
  finally:

    # Reset GPIO settings
    ####VERY IMPORTANT, ESPECIALLY IF ACTIVE PWM SIGNALS BEING USED
    GPIO.cleanup()

# Run main if this is head program
if __name__=="__main__":

  main()