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
import winsound

# ####################################INPUTS#############################################
location_marker = 'Therapist Center Hand L'
origin_marker = 'Therapist Lateral Hand L'
a_marker = 'Therapist Lateral Wrist L'
b_marker = 'Therapist Medial Hand L'
patientGrasp = True
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

# Import the vicon trajectories as a csv. Save as a dict so that each marker can be accessed by name.
trajectories = rrc.import_vicon_markers('SupineHipKnee.csv')

# Get the average of the COM marker and treat it as if the COM of the subject doesn't move
COM = [statistics.fmean(trajectories['COM']['x']),
       statistics.fmean(trajectories['COM']['y']),
       statistics.fmean(trajectories['COM']['z'])]

# Vector Calculations
v_end_start_p, v_largestNormal_start_p, v_COM_start_p, v_incenter_com_p, COM_p, approaches, largestNormalIndex = (
    rrc.perform_vector_calculations(trajectories, location_marker, origin_marker, a_marker, b_marker))

workspacePoints = rrc.import_workspace_voxels('iiwa14WorkspaceVoxels.csv', v_end_start_p, v_incenter_com_p, toleranceAngle)

# TODO: Add plot to check approach of trajectory

valid_locations = rrc.check_location(workspacePoints, v_end_start_p, 'Checking Endpoint Location')
valid_locations = rrc.check_location(valid_locations, v_largestNormal_start_p, 'Checking Largest Normal Location')

newValidLocations = []
for location in tqdm(valid_locations, desc='Checking Safety Radius'):
    COMLocation = np.array(location)+np.array(v_COM_start_p)
    if np.linalg.norm(COMLocation) >= COM_safety_radius:
        newValidLocations.append(location)
valid_locations = newValidLocations

valid_locations = rrc.check_location_full(valid_locations, trajectories[location_marker]['rotated'],
                                     'Checking Location of Full Trajectory')
# TODO: add each voxel to a list by itself
approachKeys = rrc.import_approach_keys('iiwa14ApproachKeys.csv')
boolean_solution_tensor = rrc.import_boolean_solution_tensor('boolean solution tensor.csv')
count = 0
first_time = True
for voxel in tqdm(valid_locations, desc='checking orientations'):
    valid_location = rrc.check_approach_quick([voxel], approachKeys, boolean_solution_tensor,
                                   approaches[0], [0, 0, 0])
    if not valid_location:
        continue

    valid_location = rrc.check_approach_quick([voxel], approachKeys, boolean_solution_tensor,
                                       approaches[-1], v_end_start_p,)
    if not valid_location:
        continue
    valid_location = rrc.check_approach_quick([voxel], approachKeys, boolean_solution_tensor,
                                       approaches[largestNormalIndex], v_largestNormal_start_p)
    if not valid_location:
        continue
    valid_location = rrc.check_approach_full_quick([voxel], approachKeys, boolean_solution_tensor, approaches,
                                            trajectories[location_marker]['rotated'])
    if valid_location:
        print('Pass Index :' + str(valid_locations.index(voxel)))
        break
    else:
        if first_time:
            first_time=False
            print('Fail Index :' + str(valid_locations.index(voxel)))

print(count)
winsound.Beep(4000, 1000)

