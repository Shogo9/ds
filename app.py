import pybullet as p
import pybullet_data
import time
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)
racecar = p.loadURDF('racecar/racecar.urdf')
print(p.getNumJoints(racecar))
for i in range(p.getNumJoints(racecar)):
    print(p.getJointInfo(racecar, i))
    
steeringAngle = 0.785
steeringLinks = [4, 6]
for steer in steeringLinks:
    p.setJointMotorControl2(racecar,
                            steer,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=steeringAngle)
    
targetVelocity = 60
wheelLinks = [2, 3, 5, 7]
for wheel in wheelLinks:
    p.setJointMotorControl2(racecar,
                            wheel,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity)
    
pos, _ = p.getBasePositionAndOrientation(racecar)
posPrev = pos
pos, _ = p.getBasePositionAndOrientation(racecar)
dx = np.sqrt((pos[0] - posPrev[0])**2 + (pos[1] - posPrev[1])**2)
v = dx / 0.01

hokuyo_joint = 8

    
# Run the simulation
while True:
    p.stepSimulation()
    time.sleep(0.01)
    t += 0.01

