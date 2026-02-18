import csv
import matplotlib.pyplot as plt
import numpy as np
import statistics
import math
from stl import mesh
from mpl_toolkits import mplot3d
from tqdm import tqdm
import roboticstoolbox as rtb
import spatialmath as sm

# ####################################INPUTS#############################################
trackedMarker = 'Center Hand'
originMarker = 'Medial Wrist'
medialMarker = 'Medial Hand'
lateralMarker = 'Lateral Wrist'
patientGrasp = True
palmarOrientation = True
rightHand = True
completeness = True
createPlot = False
toleranceAngle = 90  # deg
COM_safety_radius = 500  # mm
# #######################################################################################

# TODO: Extending vertical base
#  1)Check x/y  location inside radius
#  2)check entire z for orientation

#  TODO: Rotation of robot about its x/y and therefore rotation of the trajectory about the z