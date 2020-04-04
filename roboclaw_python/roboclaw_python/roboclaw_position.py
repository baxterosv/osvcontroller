#***Before using this example the motor/controller combination must be
#***tuned and the settings saved to the Roboclaw using IonMotion.
#***The Min and Max Positions must be at least 0 and 50000

import time
from roboclaw_3 import Roboclaw

#Windows comport name
#rc = Roboclaw("COM3",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyS0",38400)


rc.Open()
address = 0x80

#rc.ResetEncoders(address)

while(1):
    
    print ("Pos 50000")
    rc.SpeedAccelDeccelPositionM1(address,500,2000,500,-5000,1)
    for i in range(0,80):
        print("Position: %d, Setpoint: %d", rc.ReadEncM1(address),0)
        #displayspeed()
        time.sleep(0.1)

    time.sleep(2)
    #rc.SetEncM1(address, -10000)
    
    print ("Pos 0")
    rc.SpeedAccelDeccelPositionM1(address,4000,4000,4000,0,1)
    for i in range(0,80):
        print(rc.ReadEncM1(address))
        #displayspeed()
        time.sleep(0.1)
  
    time.sleep(2)
