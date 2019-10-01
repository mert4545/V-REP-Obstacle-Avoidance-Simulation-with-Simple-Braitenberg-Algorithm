# -*- coding: utf-8 -*-
"""
Created on Thu Sep 19 00:39:58 2019

@author: mert
"""

#======================================= Import Required Modules Section ==================================================
import sys
sys.path.append(r"D:\V-REP Tutorials\Avoid Obstacles Robot Tutorial")
import math
import numpy as np

try:
    import vrep
except:
    print("vrep module cannot be imported properly.")
    print("Please be sure that vrep.py file together with remoteApi.dll file is are located in the current path.")
#==========================================================================================================================

#========================= Set Simulation Configuration and Connect to V-REP Environment Section ==========================
print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

""" be sure that clientID != -1. If it is -1, the reason is most probably due to the fact that a simulation is not
yet started in V-REP environment. Please start to run a simulation first. """

if clientID != -1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
#==========================================================================================================================
    
""" Start by first obtaining object handles. Those handles later on will be used to take actions
First, obtain left and right motor handles """ 

returnCode, leftMotor = vrep.simxGetObjectHandle(clientID, 
                                            "Pioneer_p3dx_leftMotor",vrep.simx_opmode_blocking)
returnCode, rightMotor = vrep.simxGetObjectHandle(clientID, 
                                            "Pioneer_p3dx_rightMotor",vrep.simx_opmode_blocking)

""" Now, sensor handles. Since there are 16 sensors on Pioneer_3dx robot, initialize an array of size 16 
to hold all sensor handles and sensor read values. """ 
sensor_handles = np.zeros(16)
sensor_read_values = np.zeros(16)

# Initialize an empty array of size 16 to hold detect status of each sensor
detectStatus = np.zeros(16)

for i in range(1, 17):
    returnCode, sensor_handle = vrep.simxGetObjectHandle(clientID, 
                                            "Pioneer_p3dx_ultrasonicSensor" + str(i), vrep.simx_opmode_blocking)
    sensor_handles[i-1] = sensor_handle
    
    # Start reading data from the proximity sensors. Use operation mode = simx_opmode_streaming.
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)
    
# Set initial default values for forward speed, right and left motor speeds of the robot
v0 = 1.5
v_l = 0
v_r = 0

# Set Initial Valeus for Braitenberg Algorithm Parameters
maxDetectionRadius = 0.5 # minimum radius for being in safe region
minSafetyDist = 0.2
braitenbergL = np.array([-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
braitenbergR = np.array([-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Set simulation run to start
simStatusCheck = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

while True:
    for i in range(1,17):
        # Keep reading data from sensors
        returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, int(sensor_handles[i-1]), vrep.simx_opmode_buffer)
        distToObject = math.sqrt(math.pow(detectedPoint[0], 2) + math.pow(detectedPoint[1], 2) + math.pow(detectedPoint[2], 2)) # Calculate distance to obstacle relative to each sensor 
		
        if (detectionState == True) and (distToObject < maxDetectionRadius):
            if (distToObject < minSafetyDist): 
                distToObject = minSafetyDist
            detectStatus[i-1] = 1-((distToObject - minSafetyDist)/(maxDetectionRadius - minSafetyDist))
        else:
            detectStatus[i-1] = 0
			
    v_l = v0
    v_r = v0
	
    for i in range(1,17):
        v_l = v_l + braitenbergL[i-1] * detectStatus[i-1]
        v_r = v_r + braitenbergR[i-1] * detectStatus[i-1]
		
		
    returnCode = vrep.simxSetJointTargetVelocity(clientID, leftMotor, v_l, vrep.simx_opmode_oneshot)
    returnCode = vrep.simxSetJointTargetVelocity(clientID, rightMotor, v_r, vrep.simx_opmode_oneshot)
    