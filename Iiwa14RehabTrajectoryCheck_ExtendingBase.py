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
import RoboticRehabTrajectoryCheckFunctionClass as rrc
from Iiwa14RehabTrajectoryCheck_v1 import workspacePoints

# ####################################INPUTS#############################################
location_marker = 'Center Hand'
origin_marker = 'Medial Wrist'
a_marker = 'Medial Hand'
b_marker = 'Lateral Wrist'
patientGrasp = True
rightHand = True
completeness = True
createPlot = False
toleranceAngle = 90  # deg
COM_safety_radius = 500  # mm
# #######################################################################################

# TODO: Extending vertical base
#  1)Check x/y  location inside radius
#  2)check entire z for approach

# Import the vicon trajectories as a csv. Save as a dict so that each marker can be accessed by name.
trajectories = rrc.import_vicon_markers('Hand Movement w COM02.csv')

v_end_start_p, v_largestNormal_start_p, v_com_start_p, v_centroid_com_p, COM_p, approaches, largestNormalIndex = (
    rrc.perform_vector_calculations(trajectories, location_marker, origin_marker, a_marker, b_marker))

workspace_points = rrc.import_workspace_voxels('iiwa14WorkspaceVoxels.csv', 0, v_centroid_com_p, toleranceAngle)
planar_workspace_points = rrc.import_workspace_pixels('iiwa14WorkspacePixels.csv', v_centroid_com_p, toleranceAngle)

# TODO: These are not the correct location check functions
valid_locations = rrc.check_location(workspacePoints, v_end_start_p, 'Checking Endpoint Location')
valid_locations = rrc.check_location(valid_locations, v_largestNormal_start_p, 'Checking Largest Normal Location')

# TODO: Can the space above/below the robot be used due to the safety radius?
new_valid_locations = []
for location in tqdm(valid_locations, desc='Checking Safety Radius'):
    com_location = np.array(location[0], location[1])+np.array([v_com_start_p[0], v_com_start_p[1]])
    if np.linalg.norm(com_location) >= COM_safety_radius:
        new_valid_locations.append(location)
validLocations = new_valid_locations