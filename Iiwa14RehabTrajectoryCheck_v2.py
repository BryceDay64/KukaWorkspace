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
createPlot = True
toleranceAngle = 90  # deg
COM_safety_radius = 500  # mm
# #######################################################################################


def check_workspace(point):
    """Given a point, this function will check if it is in the Iiwa 14's workspace and will return a bool."""
    workspace_inner_bound = 280  # mm
    workspace_major_outer_bound = 946  # mm
    workspace_minor_outer_bound = 526  # mm
    r = np.linalg.norm(point)
    if r < workspace_inner_bound:
        in_workspace = False
    else:
        azimuth = np.arccos(point[2]/r)
        inclination = np.arctan2(point[1], point[0])
        if 2*math.pi / 3 >= azimuth:
            if r <= workspace_major_outer_bound:
                in_workspace = True
            else:
                in_workspace = False
        else:
            new_center = (363.73*np.cos(inclination)+point[0],
                          363.73*np.sin(inclination)+point[1],
                          210+point[2])
            if np.linalg.norm(new_center) <= workspace_minor_outer_bound:
                in_workspace = True
            else:
                in_workspace = False
    return in_workspace


def check_location(voxels, trajectory_vector, description):
    """Given a list of voxels in the Iiwa 14's workspace and vector which relates a point in the motion's trajectory to
     the start of the trajectory, this function will return a list of voxels with which the trajectory point fit."""
    new_valid_voxels = []
    for voxel in tqdm(voxels, desc=description):
        traj_location = np.array(voxel) + np.array(trajectory_vector)
        if check_workspace(traj_location):
            new_valid_voxels.append(voxel)
    return new_valid_voxels


def check_location_full(voxels, full_trajectory, description):
    new_valid_voxels = []
    for voxel in tqdm(voxels, desc=description):
        full_trajectory_bool = True
        for trajectory_point in full_trajectory[1:-2]:
            trajectory_vector = np.array(trajectory_point)-np.array(trajectories[trackedMarker]['rotated'][0])
            trajectory_in_workspace = np.array(location) + np.array(trajectory_vector)
            if not check_workspace(trajectory_in_workspace):
                full_trajectory_bool = False
                break
        if full_trajectory_bool:
            new_valid_voxels.append(voxel)
    return new_valid_voxels


def check_orientation(voxels, keys, boolean_tensor, orientation_vector, trajectory_vector, description):
    new_valid_voxels = []
    for voxel in tqdm(voxels, desc=description):
        traj_location = np.array(voxel)+np.array(trajectory_vector)
        z_index = (np.abs(np.array(keys['z'])-traj_location[2])).argmin()
        x_index = (np.abs(np.array(keys['x'])-np.linalg.norm(np.array([traj_location[0], traj_location[1]])))).argmin()
        angle = np.arctan2(traj_location[1], traj_location[0])
        approach_keys_rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                                 [np.sin(angle), np.cos(angle), 0],
                                                 [0, 0, 1]])
        rotated_approach_keys = []
        for key in keys['a']:
            rotation_of_keys = np.matmul(np.array(key), approach_keys_rotation_matrix)
            rotated_approach_keys.append(rotation_of_keys)
        a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys) - np.array(orientation_vector),
                                         axis=1))).argmin()
        if a_index >= 500:
            a_index -= 500
        if boolean_tensor[x_index][z_index][a_index]:
            new_valid_voxels.append(location)
    return new_valid_voxels


def check_orientation_full(voxels, keys, boolean_tensor, full_orientations, full_trajectory, description):
    new_valid_voxels = []
    for voxel in tqdm(voxels, desc=description):
        full_trajectory_bool = True
        for orientation_index in range(len(full_orientations)):
            traj_location = np.array(voxel) + np.array(full_trajectory[orientation_index])
            z_index = (np.abs(np.array(keys['z'])-traj_location[2])).argmin()
            x_index = (np.abs(np.array(keys['x'])
                              - np.linalg.norm(np.array([traj_location[0], traj_location[1]])))).argmin()
            angle = np.arctan2(traj_location[1], traj_location[0])
            approach_keys_rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                                     [np.sin(angle), np.cos(angle), 0],
                                                     [0, 0, 1]])
            rotated_approach_keys = []
            for key in keys['a']:
                rotation_of_keys = np.matmul(np.array(key), approach_keys_rotation_matrix)
                rotated_approach_keys.append(rotation_of_keys)
            a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys)
                                             - np.array(full_orientations[orientation_index]), axis=1))).argmin()
            if a_index >= 500:
                a_index -= 500
            if not boolean_tensor[x_index][z_index][a_index]:
                full_trajectory_bool = False
                break
        if full_trajectory_bool:
            newValidLocations.append(location)
    return new_valid_voxels


# Import the vicon trajectories as a csv. Save as a dict so that each marker can be accessed by name.
trajectories = {}
with open('Hand Movement w COM02.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    csvreader = list(csvreader)
    markers = list(filter(None, csvreader[2]))
    csvreader = csvreader[5:-1]
    for marker in tqdm(range(len(markers)), desc='Split up Markers'):
        x = []
        y = []
        z = []
        full = []
        for row in csvreader:
            x.append(float(row[3*marker+2]))
            y.append(float(row[3*marker+3]))
            z.append(float(row[3*marker+4]))
            full.append([float(row[3*marker+2]), float(row[3*marker+3]), float(row[3*marker+4])])
        newName = markers[marker].split(':')[1]
        trajectories.update({newName: {'x': x,
                                       'y': y,
                                       'z': z,
                                       'full': full}})

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
    ax.scatter(trajectories[trackedMarker]['x'],
               trajectories[trackedMarker]['y'],
               trajectories[trackedMarker]['z'], '-')
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
    x = [w - COM[0] for w in trajectories[trackedMarker]['x']]
    y = [w - COM[1] for w in trajectories[trackedMarker]['y']]
    z = [w - COM[2] for w in trajectories[trackedMarker]['z']]
    axes.scatter(x, y, z, c='m')
    axes.set_aspect('equal')
    # Show the plot to the screen
    plt.show()

# Vector Calculations
v_end_start = (np.array(trajectories[trackedMarker]['full'][-1])
               - np.array(trajectories[trackedMarker]['full'][0]))
mag_end_start = np.linalg.norm(v_end_start)
h_end_start = np.abs(v_end_start[2])
z_cutoff = h_end_start-736

largestDistance = 0
largestNormalIndex = int(round(len(trajectories[trackedMarker]['full'])/2))
for location_num in range(len(trajectories[trackedMarker]['full'])):
    v_start_traj = (np.array(trajectories[trackedMarker]['full'][0])
                    - np.array(trajectories[trackedMarker]['full'][location_num]))
    distance = ((np.linalg.norm(np.cross(v_end_start, v_start_traj))) / mag_end_start)
    if distance > largestDistance:
        largestDistance = distance
        largestNormalIndex = location_num

# Calculate the centroid of the trajectory
centroid_location = [statistics.fmean(trajectories[trackedMarker]['x']),
                     statistics.fmean(trajectories[trackedMarker]['y']),
                     statistics.fmean(trajectories[trackedMarker]['z'])]

v_centroid_start = np.array(centroid_location)-np.array(trajectories[trackedMarker]['full'][0])

v_com_start = np.array(COM)-np.array(trajectories[trackedMarker]['full'][0])
v_centroid_com = np.array(v_centroid_start) - np.array(v_com_start)
ang_COM_org_x = np.arctan2(COM[1], COM[0])
ang_incenter_com_x = np.arctan2(v_centroid_com[1], v_centroid_com[0])

# Rotates all points in trajectory about arbitrary orign to align COM with x
trajectory_rotation_matrix_org = np.array([[np.cos(ang_COM_org_x), -np.sin(ang_COM_org_x), 0],
                                           [np.sin(ang_COM_org_x), np.cos(ang_COM_org_x), 0],
                                           [0, 0, 1]])
# Rotates all points in trajectory about COM to align centroid with x
trajectory_rotation_matrix_COM = np.array([[np.cos(ang_incenter_com_x), -np.sin(ang_incenter_com_x), 0],
                                           [np.sin(ang_incenter_com_x), np.cos(ang_incenter_com_x), 0],
                                           [0, 0, 1]])
# Rotates all points in trajectory about COM to align centroid with x and COM
trajectory_rotation_matrix_fix = np.array([[np.cos(-ang_COM_org_x), -np.sin(-ang_COM_org_x), 0],
                                           [np.sin(-ang_COM_org_x), np.cos(-ang_COM_org_x), 0],
                                           [0, 0, 1]])

COM_p = np.matmul(np.array(COM), trajectory_rotation_matrix_org)

for marker in trajectories:
    alignedTrajectory = []
    for trajectory in trajectories[marker]['full']:
        rotAboutCOM = np.matmul(np.array(trajectory)-np.array(COM), trajectory_rotation_matrix_COM)
        rotAboutOrg = np.matmul(rotAboutCOM+np.array(COM), trajectory_rotation_matrix_org)
        rotToFix = np.matmul(rotAboutOrg, trajectory_rotation_matrix_fix)
        alignedTrajectory.append(rotToFix)
    trajectories[marker]['rotated'] = alignedTrajectory
v_end_start_p = np.array(trajectories[trackedMarker]['rotated'][-1])-np.array(trajectories[trackedMarker]['rotated'][0])
v_largestNormal_start_p = (np.array(trajectories[trackedMarker]['rotated'][largestNormalIndex])
                           - np.array(trajectories[trackedMarker]['rotated'][0]))
v_COM_start_p = np.array(COM_p)-np.array(trajectories[trackedMarker]['rotated'][0])
rot_incenter_COM = np.matmul(v_centroid_com, trajectory_rotation_matrix_COM)
rot_incenter_org = np.matmul(rot_incenter_COM+np.array(COM_p), trajectory_rotation_matrix_org)-COM_p
v_incenter_com_p = np.matmul(rot_incenter_org+np.array(COM_p), trajectory_rotation_matrix_fix)-COM_p

orientations = []
for index in range(len(trajectories[trackedMarker]['rotated'])):
    medialVector = (np.array(trajectories[medialMarker]['rotated'][index])
                    - np.array(trajectories[originMarker]['rotated'][index]))
    lateralVector = (np.array(trajectories[lateralMarker]['rotated'][index])
                     - np.array(trajectories[originMarker]['rotated'][index]))
    if palmarOrientation and rightHand:
        orientationVector = np.cross(np.array(medialVector), np.array(lateralVector))
        orientationUnitVector = orientationVector/np.linalg.norm(orientationVector)
        orientations.append(orientationUnitVector)

rotated_x = []
rotated_y = []
rotated_z = []
for trajectory in trajectories[trackedMarker]['rotated']:
    rotated_x.append(trajectory[0])
    rotated_y.append(trajectory[1])
    rotated_z.append(trajectory[2]-COM_p[2])

if createPlot:
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(xs=COM_p[0], ys=COM_p[1], zs=0)
    ax.scatter(xs=v_incenter_com_p[0]+COM_p[0], ys=v_incenter_com_p[1]+COM_p[1], zs=0)
    ax.scatter(xs=v_incenter_com_p[0]+COM_p[0], ys=v_incenter_com_p[1]+COM_p[1], zs=v_incenter_com_p[2])
    ax.scatter(xs=rotated_x, ys=rotated_y, zs=rotated_z)
    ax.scatter(xs=range(1500), ys=[0]*1500, zs=[0]*1500, style='--')
    ax.set_aspect('equal')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

z_location_divisions = 168
x_location_divisions = 189

workspace_width = 1892  # mm
workspace_height = 1682  # mm


workspace_locations = []
workspace_shape = []

with open('iiwa14WorkspaceVoxels.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    csvreader = list(csvreader)
    csvreader = csvreader[1:-1]
    workspacePoints = []
    for location in tqdm(csvreader, desc='Import Voxels'):
        if float(location[2]) > z_cutoff:
            if v_incenter_com_p[0] >= 0:
                if np.abs(np.rad2deg(np.arctan2(float(location[1]), float(location[0])))) >= toleranceAngle:
                    workspacePoints.append([float(location[0]), float(location[1]), float(location[2])])
            else:
                if np.abs(np.rad2deg(np.arctan2(float(location[1]), float(location[0])))) <= toleranceAngle:
                    workspacePoints.append([float(location[0]), float(location[1]), float(location[2])])

# TODO: Add plot to check orientation of trajectory

valid_locations = check_location(workspacePoints, v_end_start_p, 'Checking Endpoint Location')
valid_locations = check_location(valid_locations, v_largestNormal_start_p, 'Checking Largest Normal Location')

newValidLocations = []
for location in tqdm(valid_locations, desc='Checking Safety Radius'):
    COMLocation = np.array(location)+np.array(v_COM_start_p)
    if np.linalg.norm(COMLocation) >= COM_safety_radius:
        newValidLocations.append(location)
validLocations = newValidLocations

with open("Iiwa14OrientationKeys.csv", "r", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        stringOrientationKeys = row

# noinspection PyTypeChecker
x = list(map(float, stringOrientationKeys['x'].replace("[", "").replace("]", "").split(', ')))
# noinspection PyTypeChecker
z = list(map(float, stringOrientationKeys['z'].replace("[", "").replace("]", "").split(', ')))
# noinspection PyTypeChecker
a = list(map(float, stringOrientationKeys['a'].replace("[", "").replace("]", "")
             .replace("(", "").replace(")", "").split(', ')))

x = np.array(x)*1000
z = list((np.array(z)*1000)-360)

inner = []
outer = []
i = 0
for num in a:
    inner.append(num)
    if i == 2:
        outer.append(inner)
        inner = []
        i = 0
    else:
        i += 1
a = outer
a_flipped = []
for approach in tqdm(a, desc='Flipping approaches'):
    a_flipped.append([approach[0], -approach[1], approach[2]])
a.extend(a_flipped)

orientationKeys = {'x': x, 'z': z, 'a': a}

boolean_solution_tensor = []
boolean_solution_matrix = []
boolean_solution_vector = []
with open('boolean solution tensor.csv', 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    csvreader = list(csvreader)
    for row in tqdm(csvreader, desc='Load Orientations Tensor'):
        for item in row:
            boolean_list = list(map(int, item.replace("[", "").replace("]", "").split(', ')))
            boolean_solution_matrix.append(boolean_list)
        boolean_solution_tensor.append(boolean_solution_matrix)
        boolean_solution_matrix = []

validLocations = check_orientation(validLocations, orientationKeys, boolean_solution_tensor,
                                   orientations[0], [0, 0, 0], 'Checking Orientation of Start')

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

    '''# Set the COM of the trajectory to the origin of joint 2 to best visualize scale
    x = [w - COM[0] for w in trajectories[trackedMarker]['x']]
    y = [w - COM[1] for w in trajectories[trackedMarker]['y']]
    z = [w - COM[2] for w in trajectories[trackedMarker]['z']]
    axes.scatter(x, y, z, c='m')'''

    lbr = rtb.models.URDF.LBR()  # instantiate robot model

    T = lbr.fkine(lbr.qz, end='tool0')
    # Tep = sm.SE3.Trans(0.3, 0, 0.36) * sm.SE3.OA([0, 1, 0], [-1, 0, 0])
    # Tep = sm.SE3.Trans(0.3, 0, 0.36) * sm.SE3.OA([0, -1, 0], [1, 0, 0])
    o_vector = (np.array(trajectories[medialMarker]['rotated'][0])
                - np.array(trajectories[originMarker]['rotated'][0]))
    Tep = (sm.SE3.Trans(validLocations[0][0],
                        validLocations[0][1],
                        validLocations[0][2])
           * sm.SE3.OA([o_vector[0],
                        o_vector[1],
                        o_vector[2]],
                       [orientations[0][0],
                        orientations[0][1],
                        orientations[0][1]]))

    sol = lbr.ik_LM(Tep, joint_limits=True)

    print(sol)
    print(sol[0]*180/np.pi)

    qt = rtb.jtraj(lbr.qr, sol[0], 50)
    lbr.plot(qt.q, backend='pyplot')

    axes.set_aspect('equal')
    # Show the plot to the screen
    plt.show()

validLocations = check_orientation(validLocations, orientationKeys, boolean_solution_tensor,
                                   orientations[-1], v_end_start_p, 'Checking Orientation of End')
validLocations = check_orientation(validLocations, orientationKeys, boolean_solution_tensor,
                                   orientations[largestNormalIndex], v_largestNormal_start_p,
                                   'Checking Orientation of Largest Normal')

validLocations = check_location_full(validLocations, trajectories[trackedMarker]['rotated'],
                                     'Checking Full Trajectory Locationally')
validLocations = check_orientation_full(validLocations, orientationKeys, boolean_solution_tensor, orientations,
                                        trajectories[trackedMarker]['rotated'],
                                        'Checking Full Trajectory Orientationally')

print(len(validLocations))


# TODO: Add robotics toolbox to plot stick diagram to check orientations
