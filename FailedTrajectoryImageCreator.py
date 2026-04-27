import csv
import matplotlib.pyplot as plt
import numpy as np
import statistics
import math
from stl import mesh
from mpl_toolkits import mplot3d
from tqdm import tqdm
# import roboticstoolbox as rtb
# import spatialmath as sm
import RoboticRehabTrajectoryCheckFunctionClass as rrc
import statistics

# ####################################INPUTS#############################################
location_marker = 'Therapist Center Hand R'
origin_marker = 'Therapist Lateral Hand R'
a_marker = 'Therapist Medial Hand R'
b_marker = 'Therapist Lateral Wrist R'
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
trajectories = rrc.import_vicon_markers('uprightd1-1.csv')

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

approachKeys = rrc.import_approach_keys('iiwa14ApproachKeys.csv')
boolean_solution_tensor = rrc.import_boolean_solution_tensor('boolean solution tensor.csv')
#
# min_location_fails = 99999999
# new_valid_voxels = []
# valid_locations_indexes = []
# for voxel in tqdm(valid_locations, desc='Finding Best Fail'):
#     location_fails = 0
#     for approach_index in range(len(approaches)):
#         traj_vector = (np.array(trajectories[location_marker]['rotated'][approach_index])
#                  - np.array(trajectories[location_marker]['rotated'][0]))
#         traj_location = np.array(voxel) + traj_vector
#         z_index = (np.abs(np.array(approachKeys['z'])-traj_location[2])).argmin()
#         x_index = (np.abs(np.array(approachKeys['x'])
#                           - np.linalg.norm(np.array([traj_location[0], traj_location[1]])))).argmin()
#         angle = np.arctan2(traj_location[1], traj_location[0])
#         approach_keys_rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
#                                                  [np.sin(angle), np.cos(angle), 0],
#                                                  [0, 0, 1]])
#         rotated_approach_keys = []
#         for key in approachKeys['a']:
#             rotation_of_keys = np.matmul(np.array(key), approach_keys_rotation_matrix)
#             rotated_approach_keys.append(rotation_of_keys)
#         a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys)
#                                          - np.array(approaches[approach_index]), axis=1))).argmin()
#         if a_index >= 500:
#             a_index -= 500
#         if not boolean_solution_tensor[x_index][z_index][a_index]:
#             full_trajectory_bool = False
#             location_fails += 1
#     if location_fails == 0:
#         valid_locations_indexes.append(valid_locations.index(voxel))
#     elif location_fails < min_location_fails:
#         min_location_fails = location_fails
#         best_fail_index = valid_locations.index(voxel)
# print(valid_locations_indexes)
# print(best_fail_index)
'''valid_locations_indexes = [16898, 17679, 18453]
best_fail_index = 13864'''
start_vector = 150*np.array(approaches[0])
end_vector = 150*np.array(approaches[-1])
largest_normal_vector = 150*np.array(approaches[largestNormalIndex])

# indexes_to_graph = [13864, 16898, 17679, 18453]
#full_valid_index = 39405
random_fail_index = 100

bad_traj_start_location = valid_locations[random_fail_index]
bad_traj_ln_location = ((np.array(trajectories[location_marker]['rotated'][largestNormalIndex])
                        - np.array(trajectories[location_marker]['rotated'][0]))
                        + np.array(valid_locations[random_fail_index]))
bad_traj_end_location = ((np.array(trajectories[location_marker]['rotated'][-1])
                        - np.array(trajectories[location_marker]['rotated'][0]))
                        + np.array(valid_locations[random_fail_index]))
# good_traj_start_location = valid_locations[full_valid_index]
# good_traj_ln_location = ((np.array(trajectories[location_marker]['rotated'][largestNormalIndex])
#                         - np.array(trajectories[location_marker]['rotated'][0]))
#                         + np.array(valid_locations[full_valid_index]))
# good_traj_end_location = ((np.array(trajectories[location_marker]['rotated'][-1])
#                         - np.array(trajectories[location_marker]['rotated'][0]))
#                         + np.array(valid_locations[full_valid_index]))

full_trajectory = []
invalid_point = []
valid_point = []
for approach_index in tqdm(range(len(approaches)), desc='Sorting points'):
    traj_vector = (np.array(trajectories[location_marker]['rotated'][approach_index])
                   - np.array(trajectories[location_marker]['rotated'][0]))
    traj_location = np.array(valid_locations[random_fail_index]) + traj_vector
    z_index = (np.abs(np.array(approachKeys['z'])-traj_location[2])).argmin()
    x_index = (np.abs(np.array(approachKeys['x'])
                      - np.linalg.norm(np.array([traj_location[0], traj_location[1]])))).argmin()
    angle = np.arctan2(traj_location[1], traj_location[0])
    approach_keys_rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                             [np.sin(angle), np.cos(angle), 0],
                                             [0, 0, 1]])
    rotated_approach_keys = []
    for key in approachKeys['a']:
        rotation_of_keys = np.matmul(np.array(key), approach_keys_rotation_matrix)
        rotated_approach_keys.append(rotation_of_keys)
    a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys)
                                     - np.array(approaches[approach_index]), axis=1))).argmin()
    if a_index >= 500:
        a_index -= 500
    if not boolean_solution_tensor[x_index][z_index][a_index]:
        invalid_point.append(traj_location)
    else:
        valid_point.append(traj_location)

# for approach_index in tqdm(range(len(approaches)), desc='Sorting points'):
#     traj_vector = (np.array(trajectories[location_marker]['rotated'][approach_index])
#                    - np.array(trajectories[location_marker]['rotated'][0]))
#     traj_location = np.array(valid_locations[full_valid_index]) + traj_vector
#     full_trajectory.append(traj_location)

invalid_x = []
invalid_y = []
invalid_z = []
valid_x = []
valid_y = []
valid_z = []
full_x = []
full_y = []
full_z = []

for point in invalid_point:
    invalid_x.append(point[0])
    invalid_y.append(point[1])
    invalid_z.append(point[2])
for point in valid_point:
    valid_x.append(point[0])
    valid_y.append(point[1])
    valid_z.append(point[2])
# for point in full_trajectory:
#     full_x.append(point[0])
#     full_y.append(point[1])
#     full_z.append(point[2])


figure = plt.figure()
axes = figure.add_subplot(projection='3d')

# Load the STL files for the Iiwa 14's workspace
workspace_mesh = mesh.Mesh.from_file(r'Iiwa 14 Workspace.stl')
workspace_mesh.x -= 946
workspace_mesh.y -= 946
workspace_mesh.z -= 736
poly_collection = mplot3d.art3d.Poly3DCollection(workspace_mesh.vectors, alpha=0.2)
poly_collection.set_color((0.75, 0.75, 0.75))  # play with color
axes.add_collection3d(poly_collection)
axes.scatter(xs=valid_x, ys=valid_y, zs=valid_z, c='k', s=1)  # label='Valid Approaches'
axes.scatter(xs=invalid_x, ys=invalid_y, zs=invalid_z, c='r', s=1)  # label='Invalid Approaches'
# axes.scatter(xs=full_x, ys=full_y, zs=full_z, c='k', s=1)
axes.quiver(bad_traj_start_location[0],
            bad_traj_start_location[1],
            bad_traj_start_location[2],
            start_vector[0], start_vector[1], start_vector[2], color='c')
axes.quiver(bad_traj_ln_location[0],
            bad_traj_ln_location[1],
            bad_traj_ln_location[2],
            largest_normal_vector[0], largest_normal_vector[1], largest_normal_vector[2], color='c')
axes.quiver(bad_traj_end_location[0],
            bad_traj_end_location[1],
            bad_traj_end_location[2],
            end_vector[0], end_vector[1], end_vector[2], color='c')
'''axes.quiver(good_traj_start_location[0],
            good_traj_start_location[1],
            good_traj_start_location[2],
            start_vector[0], start_vector[1], start_vector[2], color='c')
axes.quiver(good_traj_ln_location[0],
            good_traj_ln_location[1],
            good_traj_ln_location[2],
            largest_normal_vector[0], largest_normal_vector[1], largest_normal_vector[2], color='c')
axes.quiver(good_traj_end_location[0],
            good_traj_end_location[1],
            good_traj_end_location[2],
            end_vector[0], end_vector[1], end_vector[2], color='c')'''
axes.view_init(elev=30, azim=315, roll=0)
# axes.tick_params(axis='both', which='major', labelsize=14)
# axes.label_params(axis='both', which='major', labelsize=14)
plt.rc('axes', labelsize=1000)
axes.set_aspect('equal')
axes.set_xlabel('x (mm)')
axes.set_ylabel('y (mm)')
axes.set_zlabel('z (mm)')
axes.set_xticks(np.arange(-946, 947, 946))
axes.set_yticks(np.arange(-946, 947, 946))
axes.set_zticks(np.array([-726, 0, 926]))
# axes.legend()
# Show the plot to the screen
plt.show()