import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,0)
# planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("myurdfwithslider.urdf",cubeStartPos, cubeStartOrientation)
# cid = p.createConstraint(boxId, -1, boxId, 3, p.JOINT_FIXED, [0, 0, 0], [0, 0, 3], [0, 0, 0])

cid = p.createConstraint(boxId, -1, boxId, 3, p.JOINT_FIXED, [0, 0, 0], [0, 0, 2], [0, 0, 0])


p.enableJointForceTorqueSensor(boxId, 0)

# robot1 = p.loadURDF("myurdf1.urdf",cubeStartPos, cubeStartOrientation, useMaximalCoordinates=True)
# robot2 = p.loadURDF("myurdf2.urdf",cubeStartPos, cubeStartOrientation, useMaximalCoordinates=True)

# cid = p.createConstraint(robot1, -1, robot2, 1, p.JOINT_FIXED, [1, 0, 0], [0, 0, 0], [0, 0, 0])

#print(boxId)

#nJoints = p.getNumJoints(boxId)

# print(nJoints)

# jointNameToId = {}
# for i in range(nJoints):
#   jointInfo = p.getJointInfo(boxId, i)
#   jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

# Hinge1 = jointNameToId['Hinge1']
# Hinge2 = jointNameToId['Hinge2']
# Hinge3 = jointNameToId['Hinge3']

# body3 = jointNameToId['body3']
# body4 = jointNameToId['body4']

#cid = p.createConstraint(0, 2, 0, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
# st = p.getLinkState(boxId, 3)
# print("----------")
# print(st[0])
# print("----------")

for i in range (1000000):
  st = p.getLinkState(boxId, 2)
  # print(st[0])
  p.stepSimulation()
  time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()  
# p.removeConstraint(cid)