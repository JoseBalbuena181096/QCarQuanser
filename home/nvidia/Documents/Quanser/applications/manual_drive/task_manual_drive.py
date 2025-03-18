## task_task_manual_drive.py
# This example demonstrates how to use the LogitechF710 to send throttle and steering 
# commands to the QCar depending on 2 driving styles. 
# Use the hardware_test_basic_io.py to troubleshoot uses trying to drive the QCar.

from pal.utilities.lidar import Lidar 
from pal.products.qcar import QCar
from pal.utilities.gamepad import LogitechF710
from pal.utilities.math import *

import os
import time
import struct
import numpy as np 

import cv2

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Timing Parameters and methods 
startTime = time.time()
def elapsed_time():
    return time.time() - startTime

sampleRate     = 50
sampleTime     = 1/sampleRate
simulationTime = 60.0
print('Sample Time: ', sampleTime)

# Additional parameters
counter = 0

# Initialize motor command array
QCarCommand = np.array([0,0])

# Set up a differentiator to get encoderSpeed from encoderCounts
diff = Calculus().differentiator_variable(sampleTime)
_ = next(diff)
timeStep = sampleTime

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## QCar and Gamepad Initialization
# Specify is using virtual or physical QCar by specifying hardware flag
# Changing readmode to 0 to use imediate I/O
readMode = 0

myCar = QCar(readMode=readMode)
gpad = LogitechF710()

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Driving Configuration: Use 3 toggles or 4 toggles mode as you see fit:
# Common to both 3 or 4 mode
#   Steering                    - Left Lateral axis
#   Arm                         - buttonLeft
# In 3 mode: 
#   Throttle (Drive or Reverse) - Right Longitudonal axis
# In 4 mode:
#   Throttle                    - Right Trigger (always positive)
#   Button A                    - Reverse if held, Drive otherwise
configuration = '3' # change to '4' if required

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Reset startTime before Main Loop
startTime = time.time()

###################################################
# Lidar settings
numMeasurements 	 = 360	# Points
lidarMeasurementMode 	 = 2
lidarInterpolationMode = 0

# Additional parameters
counter 	= 0
imageWidth  = 1640
imageHeight = 820
cameraID 	= '3'

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Additional parameters and buffers
pixelsPerMeter 	  = 50 # pixels per meter
sideLengthScale = 8 * pixelsPerMeter # 8 meters width, or 400 pixels side length
decay 		      = 0.9 # 90% decay rate on old map data
maxDistance  	  = 1.5
map    		  	  = np.zeros((sideLengthScale, sideLengthScale), dtype=np.float32) # map object

# LIDAR initialization and measurement buffers
myLidar = Lidar(
	type='RPLidar',
	numMeasurements=numMeasurements,
	rangingDistanceMode=lidarMeasurementMode, 
	interpolationMode=lidarInterpolationMode
)
##################################################

def lidar_measure(lidar, map):
    map = decay*map                                                            
	# Start  this iteration
	# Capture LIDAR data
    myLidar.read()
 	# convert angles from lidar frame to body frame
    anglesInBodyFrame = myLidar.angles * -1 + np.pi/2 
	# Find the points where it exceed the max distance and drop them off
    idx = [i for i, v in enumerate(myLidar.distances) if v < maxDistance] 
	# convert distances and angles to XY contour
    x = myLidar.distances[idx]*np.cos(anglesInBodyFrame[idx])
    y = myLidar.distances[idx]*np.sin(anglesInBodyFrame[idx]) 
    	    

	# convert XY contour to pixels contour and update those pixels in the map
    pX = (sideLengthScale/2 - x*pixelsPerMeter).astype(np.uint16)   
    pY = (sideLengthScale/2 - y*pixelsPerMeter).astype(np.uint16)	
    coordenadas_filtradas = [(px, py) for px, py in zip(pX, pY) if py <= 210 and py >= 190 and px <= 190]
    if len(coordenadas_filtradas)>0: 
        pX_filtrado, pY_filtrado = zip(*coordenadas_filtradas)
        map[pX_filtrado, pY_filtrado] = 1	
        return map, pY_filtrado, pX_filtrado
    return map, [], []



## Main Loop
try:
    while True:
        start = time.time()

        # Start timing this iteration
        #start = elapsed_time()

        # Read Gamepad states
        new = gpad.read()

        # Basic IO - write motor commands
        if configuration == '3':
            if new and gpad.buttonLeft:
                QCarCommand = np.array([0.3*gpad.rightLongitudinolAxis, 0.5*gpad.leftLateralAxis])            
        elif configuration == '4':
            if new and gpad.buttonLeft:
                if gpad.A:
                    QCarCommand = np.array([-0.3*gpad.trigger, 0.5*gpad.leftLateralAxis])
                else:
                    QCarCommand = np.array([0.3*gpad.trigger, 0.5*gpad.leftLateralAxis])            
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

        # Adjust LED indicators based on steering and reverse indicators based on reverse gear
        if QCarCommand[1] > 0.3:
            LEDs[0] = 1
            LEDs[2] = 1
        elif QCarCommand[1] < -0.3:
            LEDs[1] = 1
            LEDs[3] = 1
        if QCarCommand[0] < 0:
            LEDs[5] = 1

        # Perform I/O
        myCar.read_write_std(throttle= QCarCommand[0], 
                             steering= QCarCommand[1], 
							 LEDs= LEDs)
                      
        map_, pX_, pY_ = lidar_measure(myLidar, map)
        
        #try:
        #    cv2.imshow('Map', map_)
        #except:
        #    pass
                                       
        batteryVoltage = myCar.batteryVoltage 

        # Estimate linear speed in m/s
        encoderSpeed = myCar.motorTach
        linearSpeed  = encoderSpeed*(1/myCar.ENCODER_COUNTS_PER_REV/4)*myCar.PIN_TO_SPUR_RATIO*2*np.pi*myCar.WHEEL_RADIUS

        # End timing this iteration
        #end = elapsed_time()

        print(map_)   
        cv2.waitKey(1)

        end = time.time()
        dt = end - start
        
        # Calculate computation time, and the time that the thread should pause/sleep for
        computationTime = end - start
        sleepTime = sampleTime - computationTime%sampleTime

        # Pause/sleep and print out the current timestamp
        time.sleep(sleepTime)

        if new:
            os.system('clear')
            print("Car Speed:\t\t\t{0:1.2f}\tm/s\nRemaining battery capacity:\t{1:4.2f}\t%\nMotor throttle:\t\t\t{2:4.2f}\t% PWM\nSteering:\t\t\t{3:3.2f}\trad"
                                                            .format(linearSpeed, 100 - (batteryVoltage - 10.5)*100/(12.6 - 10.5), QCarCommand[0], QCarCommand[1]))
        timeAfterSleep = elapsed_time()
        timeStep = timeAfterSleep - start
        counter += 1



except KeyboardInterrupt:
    print("User interrupted!")

finally:    
    myCar.terminate()
    myLidar.terminate() 
    f = open("dataxd.csv", "a")
    f.write(str(map_))
    f.close()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 