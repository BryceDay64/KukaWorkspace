import numpy as np

# One Leg Reach Data ################
vAnkleToShoulderSame = np.array([78.52, 566.92, 1138.47])
vShoulderToFingerSame = np.array([-44.38, 703.2, 66.32])
vAnkleToShoulderCross = np.array([-44.85, 562.12, 1134.98])
vShoulderToFingerCross = np.array([-114.27, 709.6, 65.98])
magArm = np.linalg.norm(vShoulderToFingerSame)

# constants #########################
myHeight = 1816.1
myArmLength = 770
shortestHeight = 1496
tallestHeight = 1872
shortRatio = shortestHeight/myHeight
tallRatio = tallestHeight/myHeight

maxReachAngle = np.deg2rad(45)
vMaxReachAngle = np.array([np.sin(maxReachAngle)*np.cos(maxReachAngle),
                          np.sin(maxReachAngle)*np.sin(maxReachAngle),
                          np.cos(maxReachAngle)])
#  Calculations
vMinReach = shortRatio*vAnkleToShoulderCross+shortRatio*magArm*vMaxReachAngle
vMaxReach = tallRatio*vShoulderToFingerSame+tallRatio*magArm*np.array([0, 1, 0])
vReachDifference = vMaxReach-vMinReach

print(vMinReach)
print(vMaxReach)
print(vReachDifference)