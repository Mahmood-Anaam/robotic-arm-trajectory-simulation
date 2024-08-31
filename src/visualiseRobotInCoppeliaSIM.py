# Description: This script is used to visualise the robot in CoppeliaSim using the joint angles obtained from the inverse kinematics solution
# Usage: Use this in conjunction with the CoppeliaSim scene provided in the assignment folder
#        Run the CoppeliaSim scene and then run this script

import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from trajectories import *


print('Program started')


robot = define_robot(d=0.08)
poses = define_robot_poses()

# connect to the CoppeliaSim server
client = RemoteAPIClient()
sim = client.getObject('sim')

# get the handle of the joint to be controlled
j1_handle = sim.getObject('/Joint1')
j2_handle = sim.getObject('/Joint2')
j3_handle = sim.getObject('/Joint3')
j4_handle = sim.getObject('/Joint4')
j5_handle = sim.getObject('/Joint5')
j6_handle = sim.getObject('/Joint6')

# enable the stepping mode:
sim.setStepping(True)

# start the simulation
sim.startSimulation()

n = 30    # number of points in the trajectory 
q = generateLinearTrajectory(poses['StartPose'],poses['TestPose1'],n=n,robot=robot)


# n = 30    # number of points in the trajectory 
# q = generateLinearTrajectory(poses['TestPose2'],poses['EndPose'],n=n,robot=robot)


# n = 2
# q = jointAnglesAtTestPoints() 


# n=30 # number of points in the trajectory 
# q = generateCircularTrajectory(poses['TestPose1'],poses['TestPose2'],C=poses['CenterPointC'],n=n,robot=robot)

# assign a position to each joint
for step in range(n):
     print(f'Step: {step}')
     sim.setJointPosition(j1_handle, q[0, step])
     sim.setJointPosition(j2_handle, q[1, step])
     sim.setJointPosition(j3_handle, q[2, step])
     sim.setJointPosition(j4_handle, q[3, step])
     sim.setJointPosition(j5_handle, q[4, step])
     sim.setJointPosition(j6_handle, q[5, step])
     sim.step()
     time.sleep(0.25)

# stop the simulation
sim.stopSimulation()

print('Program ended')