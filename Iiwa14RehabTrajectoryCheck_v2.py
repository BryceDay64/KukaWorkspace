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
trajectories = rrc.import_vicon_markers('KneeROM.csv')

# Get the average of the COM marker and treat it as if the COM of the subject doesn't move
COM = [statistics.fmean(trajectories['COM']['x']),
       statistics.fmean(trajectories['COM']['y']),
       statistics.fmean(trajectories['COM']['z'])]

# Check to make sure trajectory and COM look correct
if createPlot:
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(0, 0, 0)
    ax.scatter(COM[0], COM[1], COM[2])
    ax.scatter(trajectories[location_marker]['x'],
               trajectories[location_marker]['y'],
               trajectories[location_marker]['z'], '-')
    ax.set_aspect('equal')
    plt.show()

# Check to see trajectory fit can be ruled out by observation.
if createPlot:
    figure = plt.figure()
    axes = figure.add_subplot(projection='3d')

    # Load the STL files for the Iiwa 14's workspace
    workspace_mesh = mesh.Mesh.from_file(r'Iiwa 14 Workspace.stl')
    workspace_mesh.x -= 946
    workspace_mesh.y -= 946
    workspace_mesh.z -= 736
    poly_collection = mplot3d.art3d.Poly3DCollection(workspace_mesh.vectors, alpha=0.2)
    poly_collection.set_color((0.5, 0.5, 1))  # play with color
    axes.add_collection3d(poly_collection)

    # Set the COM of the trajectory to the origin of joint 2 to best visualize scale
    x = [w - COM[0] for w in trajectories[location_marker]['x']]
    y = [w - COM[1] for w in trajectories[location_marker]['y']]
    z = [w - COM[2] for w in trajectories[location_marker]['z']]
    axes.scatter(x, y, z, c='m')
    axes.set_aspect('equal')
    # Show the plot to the screen
    plt.show()

# Vector Calculations
v_end_start_p, v_largestNormal_start_p, v_COM_start_p, v_incenter_com_p, COM_p, approaches, largestNormalIndex = (
    rrc.perform_vector_calculations(trajectories, location_marker, origin_marker, a_marker, b_marker))

if createPlot:
    rotated_x = []
    rotated_y = []
    rotated_z = []
    for trajectory in trajectories[location_marker]['rotated']:
        rotated_x.append(trajectory[0])
        rotated_y.append(trajectory[1])
        rotated_z.append(trajectory[2])

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(xs=COM_p[0], ys=COM_p[1], zs=COM_p[2])
    ax.scatter(xs=COM_p[0], ys=COM_p[1], zs=0)
    ax.scatter(xs=v_incenter_com_p[0]+COM_p[0], ys=v_incenter_com_p[1]+COM_p[1], zs=0)
    ax.scatter(xs=v_incenter_com_p[0]+COM_p[0], ys=v_incenter_com_p[1]+COM_p[1], zs=v_incenter_com_p[2]+COM_p[2])
    ax.scatter(xs=rotated_x, ys=rotated_y, zs=rotated_z)
    '''ax.scatter(xs=range(1500), ys=[0]*1500, zs=[0]*1500)'''
    ax.set_aspect('equal')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    print(COM_p)
    print(v_incenter_com_p)
    plt.show()

workspacePoints = rrc.import_workspace_voxels('iiwa14WorkspaceVoxels.csv', v_end_start_p, v_incenter_com_p, toleranceAngle)

# TODO: Add plot to check approach of trajectory

valid_locations = rrc.check_location(workspacePoints, v_end_start_p, 'Checking Endpoint Location')
valid_locations = rrc.check_location(valid_locations, v_largestNormal_start_p, 'Checking Largest Normal Location')

newValidLocations = []
for location in tqdm(valid_locations, desc='Checking Safety Radius'):
    COMLocation = np.array(location)+np.array(v_COM_start_p)
    if np.linalg.norm(COMLocation) >= COM_safety_radius:
        newValidLocations.append(location)
validLocations = newValidLocations

approachKeys = rrc.import_approach_keys('iiwa14ApproachKeys.csv')
boolean_solution_tensor = rrc.import_boolean_solution_tensor('boolean solution tensor.csv')

validLocations = rrc.check_approach(validLocations, approachKeys, boolean_solution_tensor,
                                   approaches[0], [0, 0, 0], 'Checking Approach of Start')

if createPlot:
    figure = plt.figure()
    axes = figure.add_subplot(projection='3d')

    # Load the STL files for the Iiwa 14's workspace
    workspace_mesh = mesh.Mesh.from_file(r'Iiwa 14 Workspace.stl')
    workspace_mesh.x -= 946
    workspace_mesh.y -= 946
    workspace_mesh.z -= 736
    poly_collection = mplot3d.art3d.Poly3DCollection(workspace_mesh.vectors, alpha=0.2)
    poly_collection.set_color((0.5, 0.5, 1))  # play with color
    axes.add_collection3d(poly_collection)
    axes.plot(xs=valid_locations[0][0], ys=validLocations[0][1], zs=validLocations[0][2])

    '''# Set the COM of the trajectory to the origin of joint 2 to best visualize scale
    x = [w - COM[0] for w in trajectories[trackedMarker]['x']]
    y = [w - COM[1] for w in trajectories[trackedMarker]['y']]
    z = [w - COM[2] for w in trajectories[trackedMarker]['z']]
    axes.scatter(x, y, z, c='m')'''

    lbr = rtb.models.URDF.LBR()  # instantiate robot model

    T = lbr.fkine(lbr.qz, end='tool0')
    # Tep = sm.SE3.Trans(0.3, 0, 0.36) * sm.SE3.OA([0, 1, 0], [-1, 0, 0])
    # Tep = sm.SE3.Trans(0.3, 0, 0.36) * sm.SE3.OA([0, -1, 0], [1, 0, 0])
    o_vector = (np.array(trajectories[a_marker]['rotated'][0])
                - np.array(trajectories[origin_marker]['rotated'][0]))
    Tep = (sm.SE3.Trans(validLocations[0][0],
                        validLocations[0][1],
                        validLocations[0][2])
           * sm.SE3.OA([o_vector[0],
                        o_vector[1],
                        o_vector[2]],
                       [approaches[0][0],
                        approaches[0][1],
                        approaches[0][1]]))

    sol = lbr.ik_LM(Tep, joint_limits=True)

    print(sol)
    print(sol[0]*180/np.pi)

    qt = rtb.jtraj(sol[0], sol[0], 1)
    lbr.plot(qt.q, backend='pyplot')

    axes.set_aspect('equal')
    # Show the plot to the screen
    plt.show()

validLocations = rrc.check_approach(validLocations, approachKeys, boolean_solution_tensor,
                                   approaches[-1], v_end_start_p, 'Checking Approach of End')
validLocations = rrc.check_approach(validLocations, approachKeys, boolean_solution_tensor,
                                   approaches[largestNormalIndex], v_largestNormal_start_p,
                                   'Checking Approach of Largest Normal')
validLocations = rrc.check_location_full(validLocations, trajectories[location_marker]['rotated'],
                                     'Checking Location of Full Trajectory')
validLocations = rrc.check_approach_full(validLocations, approachKeys, boolean_solution_tensor, approaches,
                                        trajectories[location_marker]['rotated'],
                                        'Checking Approach of Full Trajectory')

print(len(validLocations))
winsound.Beep(4000, 5000)

