import csv
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import statistics
import math
# import scipy.spatial as sps
# import plotly.graph_objects as go
from stl import mesh
from mpl_toolkits import mplot3d
from tqdm import tqdm
# from pyntcloud import PyntCloud
# import pyvista as pv

#####################################INPUTS#############################################
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
########################################################################################


# TODO:
#  #1: Import path as csv
#  #2: Import COM trajectory
#  #3: Create visualization in pygame? to correct for orientation actively
#  #4: Ask for rotational tollerance about COM for each axis
#  #5: Ask about passive height adjustment and height, or active height adjustment and range, ask about passive angle adjustment and start ot active and range


#  TODO:
#   #1 check create limit of possible zones based off height
#   #2 change tolerance angle to be projected vector between robot center and COM to x axis


def checkworkspace(point):
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


def vectorbetweenpoints(point1, point2):
    vector = [point2[0]-point1[0],
              point2[1]-point1[1],
              point2[2]-point1[2]]
    return vector

def check_location(locations, trajectory_point):
    newValidLocations = []
    for location in locations:
        endLocation = np.array(location) + np.array(trajectory_point)
        if checkworkspace(endLocation):
            validLocations.append(location)
    return newValidLocations


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

COM = [statistics.fmean(trajectories['COM']['x']),
       statistics.fmean(trajectories['COM']['y']),
       statistics.fmean(trajectories['COM']['z'])]

for marker in trajectories.keys():
    trajectories[marker]['x'] = [w - COM[0] for w in trajectories[marker]['x']]
    trajectories[marker]['y'] = [w - COM[1] for w in trajectories[marker]['y']]
    trajectories[marker]['z'] = [w - COM[2] for w in trajectories[marker]['z']]
if createPlot:
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(0, 0, 0)
    ax.scatter(COM[0], COM[1], COM[2])
    ax.scatter(trajectories['Center Hand']['x'],
               trajectories['Center Hand']['y'],
               trajectories['Center Hand']['z'], '-')
    ax.set_aspect('equal')
    plt.show()


# Create a new plot
if createPlot:
    figure = plt.figure()
    axes = figure.add_subplot(projection='3d')

    # Load the STL files and add the vectors to the plot
    workspace_mesh = mesh.Mesh.from_file(r'Iiwa 14 Worksace.stl')
    workspace_mesh.x -= 946
    workspace_mesh.y -= 946
    workspace_mesh.z -= 736
    poly_collection = mplot3d.art3d.Poly3DCollection(workspace_mesh.vectors, alpha=0.2)
    poly_collection.set_color((0.5, 0.5, 1))  # play with color
    axes.add_collection3d(poly_collection)

    axes.scatter(trajectories['Center Hand']['x'],
                 trajectories['Center Hand']['y'],
                 trajectories['Center Hand']['z'], c='m')
    axes.set_aspect('equal')
    # Show the plot to the screen
    plt.show()

# Vector Calculations
v_end_start = vectorbetweenpoints(trajectories[trackedMarker]['full'][0], trajectories[trackedMarker]['full'][-1])
mag_end_start = np.linalg.norm(v_end_start)
h_end_start = np.abs(v_end_start[2])
z_cutoff = h_end_start-736

largestDistance = 0
for location_num in range(len(trajectories[trackedMarker]['full'])):
    v_start_traj = vectorbetweenpoints(trajectories[trackedMarker]['full'][location_num],
                                       trajectories[trackedMarker]['full'][0])
    distance = ((np.linalg.norm(np.cross(v_end_start, v_start_traj))) / mag_end_start)
    if distance > largestDistance:
        largestDistance = distance
        largestNormalIndex = location_num
v_largestNormal_start = vectorbetweenpoints(trajectories[trackedMarker]['full'][0],
                                            trajectories[trackedMarker]['full'][largestNormalIndex])
largestNormal = largestDistance
mag_largestNormal_start = np.linalg.norm(v_largestNormal_start)

v_end_largestNormal = np.array(v_end_start)-np.array(v_largestNormal_start)
mag_end_largestNormal = np.linalg.norm(v_end_largestNormal)

v_incenter_start = [(mag_largestNormal_start * v_end_start[0] + mag_end_start * v_largestNormal_start[0]) /
                    (mag_largestNormal_start + mag_end_start + mag_end_largestNormal),
                    (mag_largestNormal_start * v_end_start[1] + mag_end_start * v_largestNormal_start[1]) /
                    (mag_largestNormal_start + mag_end_start + mag_end_largestNormal),
                    (mag_largestNormal_start * v_end_start[2] + mag_end_start * v_largestNormal_start[2]) /
                    (mag_largestNormal_start + mag_end_start + mag_end_largestNormal)]

v_com_start = vectorbetweenpoints(trajectories[trackedMarker]['full'][0], COM)
v_incenter_com = np.array(v_incenter_start) - np.array(v_com_start)
ang_COMProjXY_org_x = np.arctan2(COM[1], COM[0])
ang_incenter_com = np.arctan2(v_incenter_com[1], v_incenter_com[0])
'''ang_incenterComProjXY_org_x = np.arccos(np.dot(v_incenter_com, np.array(COM))/(
    np.linalg.norm(v_incenter_com)*np.linalg.norm(np.array(COM))))'''
ang_incenterComProjXY_org_x = np.arctan2(v_incenter_com[1], v_incenter_com[0])

# Rotates all points in trajectory about arbitrary orign to align COM with x
trajectory_rotation_matrix_org = np.array([[np.cos(ang_COMProjXY_org_x), -np.sin(ang_COMProjXY_org_x), 0],
                                           [np.sin(ang_COMProjXY_org_x), np.cos(ang_COMProjXY_org_x), 0],
                                           [0, 0, 1]])
# Rotates all points in trajectory about COM to align incenter with x
trajectory_rotation_matrix_COM = np.array([[np.cos(ang_incenterComProjXY_org_x), -np.sin(ang_incenterComProjXY_org_x), 0],
                                           [np.sin(ang_incenterComProjXY_org_x), np.cos(ang_incenterComProjXY_org_x), 0],
                                           [0, 0, 1]])
trajectory_rotation_matrix_fix = np.array([[np.cos(-ang_COMProjXY_org_x), -np.sin(-ang_COMProjXY_org_x), 0],
                                           [np.sin(-ang_COMProjXY_org_x), np.cos(-ang_COMProjXY_org_x), 0],
                                           [0, 0, 1]])

COM_p = np.matmul(np.array(COM), trajectory_rotation_matrix_org)

for marker in trajectories:
    alignedTrajectory = []
    for trajectory in trajectories[marker]['full']:
        rotAboutCOM = np.matmul(np.array(trajectory)-np.array(COM), trajectory_rotation_matrix_COM)
        rotAboutOrg = np.matmul(rotAboutCOM+np.array(COM),trajectory_rotation_matrix_org)
        rotToFix = np.matmul(rotAboutOrg, trajectory_rotation_matrix_fix)
        alignedTrajectory.append(rotToFix)
    trajectories[marker]['rotated'] = alignedTrajectory
v_end_start_p = vectorbetweenpoints(trajectories[trackedMarker]['rotated'][0],
                                    trajectories[trackedMarker]['rotated'][-1])
v_largestNormal_start_p = vectorbetweenpoints(trajectories[trackedMarker]['rotated'][0],
                                              trajectories[trackedMarker]['rotated'][largestNormalIndex])
v_COM_start_p = vectorbetweenpoints(trajectories[trackedMarker]['rotated'][0], COM_p)

rot_incenter_COM = np.matmul(v_incenter_com, trajectory_rotation_matrix_COM)
rot_incenter_org = np.matmul(rot_incenter_COM+np.array(COM_p), trajectory_rotation_matrix_org)-COM_p
v_incenter_com_p = np.matmul(rot_incenter_org+np.array(COM_p), trajectory_rotation_matrix_fix)-COM_p

orientations = []
for index in range(len(trajectories[trackedMarker]['rotated'])):
    medialVector = vectorbetweenpoints(trajectories[originMarker]['rotated'][index], trajectories[medialMarker]['rotated'][index])
    lateralVector = vectorbetweenpoints(trajectories[originMarker]['rotated'][index], trajectories[lateralMarker]['rotated'][index])
    if palmarOrientation and rightHand:
        orientationVector = np.cross(np.array(medialVector), np.array(lateralVector))
        orienationUnitVector = orientationVector/np.linalg.norm(orientationVector)
        orientations.append(orienationUnitVector)

rotated_x = []
rotated_y = []
rotated_z = []
for trajectory in trajectories[trackedMarker]['rotated']:
    rotated_x.append(trajectory[0])
    rotated_y.append(trajectory[1])
    rotated_z.append(trajectory[2]-COM_p[2])

'''print(COM_p[0])
print(v_incenter_com_p[0])'''

if createPlot:
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(xs=COM_p[0], ys=COM_p[1], zs=0)
    ax.scatter(xs=v_incenter_com_p[0]+COM_p[0], ys=v_incenter_com_p[1]+COM_p[1], zs=0)
    ax.scatter(xs=v_incenter_com_p[0]+COM_p[0], ys=v_incenter_com_p[1]+COM_p[1], zs=v_incenter_com_p[2])
    ax.scatter(xs=rotated_x, ys=rotated_y, zs=rotated_z)
    '''ax.scatter(xs=range(1500), ys=[0]*1500, zs=[0]*1500)'''
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
# TODO: COM check for collision

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

# check if start and end can fit at given location

validLocations = []
for location in tqdm(workspacePoints, desc='Checking Endpoint Location'):
    endLocation = np.array(location)+np.array(v_end_start_p)
    if checkworkspace(endLocation):
        validLocations.append(location)

newValidLocations = []
for location in tqdm(validLocations, desc='Checking Largest Normal Location'):
    firstNormalLocation = np.array(location)+np.array(v_largestNormal_start_p)
    if checkworkspace(firstNormalLocation):
        newValidLocations.append(location)
validLocations = newValidLocations

'''newValidLocations = []
i = 0
originFromStart = np.negative(np.array(trajectories[trackedMarker]['full'][0]))
largestNormalVectorProjected = [largestNormalVector[0], largestNormalVector[1]]
for location in validLocations:
    i += 1
    print('Checking Tolerance Angle:' + str(i) + '/' + str(len(validLocations)))
    negativeTrajectoryOrigin = np.negative(np.array(location) + originFromStart)
    negativeTrajectoryOriginProjected = [negativeTrajectoryOrigin[0], negativeTrajectoryOrigin[1]]
    angleBetweenVectors = np.abs(np.arccos(np.dot(largestNormalVectorProjected, negativeTrajectoryOriginProjected)/(
            np.linalg.norm(largestNormalVectorProjected)*np.linalg.norm(negativeTrajectoryOriginProjected))))
    print(np.rad2deg(angleBetweenVectors))
    if angleBetweenVectors <= np.deg2rad(toleranceAngle):
        newValidLocations.append(location)
validLocations = newValidLocations'''

newValidLocations = []
for location in tqdm(validLocations, desc='Checking Safety Radius'):
    COMLocation = np.array(location)+np.array(v_COM_start_p)
    if np.linalg.norm(COMLocation) >= COM_safety_radius:
        newValidLocations.append(location)
validLocations = newValidLocations

with open("Iiwa14OrientationKeys.csv", "r", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        stringOrientationKeys = row

x = list(map(float, stringOrientationKeys['x'].replace("[", "").replace("]", "").split(', ')))
z = list(map(float, stringOrientationKeys['z'].replace("[", "").replace("]", "").split(', ')))
a = list(map(float, stringOrientationKeys['a'].replace("[", "").replace("]", "").replace("(", "").replace(")", "").split(', ')))

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

# TODO: Check if approach vector needs to be negative and determine how to mirror hemisphere
newValidLocations = []
for location in tqdm(validLocations, desc='Checking Orientation of Start'):
    z_index = (np.abs(np.array(orientationKeys['z'])-location[2])).argmin()
    x_index = (np.abs(np.array(orientationKeys['x'])-np.linalg.norm(np.array([location[0], location[1]])))).argmin()
    angle = np.arctan2(location[1], location[0])
    approachKeys_rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                             [np.sin(angle), np.cos(angle), 0],
                                             [0, 0, 1]])
    rotated_approach_keys =[]
    for key in orientationKeys['a']:
        rotApproachKeys = np.matmul(np.array(key), approachKeys_rotation_matrix)
        rotated_approach_keys.append(rotApproachKeys)
    a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys)-np.array(orientations[0]), axis=1))).argmin()
    if a_index >= 500:
        a_index -= 500
    if boolean_solution_tensor[x_index][z_index][a_index]:
        newValidLocations.append(location)
validLocations = newValidLocations

newValidLocations = []
for location in tqdm(validLocations, desc='Checking Orientation of End'):
    z_index = (np.abs(np.array(orientationKeys['z'])-location[2])).argmin()
    x_index = (np.abs(np.array(orientationKeys['x'])-np.linalg.norm(np.array([location[0], location[1]])))).argmin()
    angle = np.arctan2(location[1], location[0])
    approachKeys_rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                             [np.sin(angle), np.cos(angle), 0],
                                             [0, 0, 1]])
    rotated_approach_keys =[]
    for key in orientationKeys['a']:
        rotApproachKeys = np.matmul(np.array(key), approachKeys_rotation_matrix)
        rotated_approach_keys.append(rotApproachKeys)
    a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys)-np.array(orientations[-1]), axis=1))).argmin()
    if a_index >= 500:
        a_index -= 500
    if boolean_solution_tensor[x_index][z_index][a_index]:
        newValidLocations.append(location)
validLocations = newValidLocations

newValidLocations = []
for location in tqdm(validLocations, desc='Checking Orientation of Largest Normal'):
    z_index = (np.abs(np.array(orientationKeys['z'])-location[2])).argmin()
    x_index = (np.abs(np.array(orientationKeys['x'])-np.linalg.norm(np.array([location[0], location[1]])))).argmin()
    angle = np.arctan2(location[1], location[0])
    approachKeys_rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                             [np.sin(angle), np.cos(angle), 0],
                                             [0, 0, 1]])
    rotated_approach_keys =[]
    for key in orientationKeys['a']:
        rotApproachKeys = np.matmul(np.array(key), approachKeys_rotation_matrix)
        rotated_approach_keys.append(rotApproachKeys)
    a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys)-np.array(orientations[largestNormalIndex]), axis=1))).argmin()
    if a_index >= 500:
        a_index -= 500
    if boolean_solution_tensor[x_index][z_index][a_index]:
        newValidLocations.append(location)
validLocations = newValidLocations

newValidLocations = []
for location in tqdm(validLocations, desc='Checking Full Trajectory Locationally'):
    fullTrajectory = True
    for trajectory in trajectories[trackedMarker]['rotated'][1:-2]:
        trajectoryVector = vectorbetweenpoints(trajectories[trackedMarker]['rotated'][0], trajectory)
        trajectoryInWorkspace = np.array(location)+np.array(trajectoryVector)
        if not checkworkspace(trajectoryInWorkspace):
            fullTrajectory = False
            break
    if fullTrajectory:
        newValidLocations.append(location)
validLocations = newValidLocations

newValidLocations = []
for location in tqdm(validLocations, desc='Checking Full Trajectory Orientationally'):
    fullTrajectory = True
    z_index = (np.abs(np.array(orientationKeys['z'])-location[2])).argmin()
    x_index = (np.abs(np.array(orientationKeys['x'])-np.linalg.norm(np.array([location[0], location[1]])))).argmin()
    angle = np.arctan2(location[1], location[0])
    approachKeys_rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                             [np.sin(angle), np.cos(angle), 0],
                                             [0, 0, 1]])
    rotated_approach_keys =[]
    for key in orientationKeys['a']:
        rotApproachKeys = np.matmul(np.array(key), approachKeys_rotation_matrix)
        rotated_approach_keys.append(rotApproachKeys)
    for orientation in orientations:
        a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys)-np.array(orientation), axis=1))).argmin()
        if a_index >= 500:
            a_index -= 500
        if not boolean_solution_tensor[x_index][z_index][a_index]:
            fullTrajectory = False
            break
    if fullTrajectory:
        newValidLocations.append(location)
validLocations = newValidLocations

'''# TODO: THIS DOESN'T WORK IT IS JUST COPY AND PASTED FROM ABOVE AS PLACEHOLDER
orientations = {}
with open('iiwa141762967540628.9177.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    csvreader = list(csvreader)
    current_x = tuple(map(float, csvreader[1][0].replace("(", "").replace(")", "").split(', ')))[0]
    current_z = tuple(map(float, csvreader[1][0].replace("(", "").replace(")", "").split(', ')))[2]
    z = []
    x = []
    o = []
    boolean_tensor = []
    boolean_tensor_mid = []
    boolean_tensor_inner = []
    for row in range(1, len(csvreader)):
        print('load Orientation Look-Up: ' + str(row) + " / " + str(len(csvreader)))



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

        csvreader = csv.reader(csvfile)
        csvreader = list(csvreader)
        current_location = [csvreader[1][0][0], csvreader[1][0][2]]


            x = tuple(map(float, csvreader[row][0].replace("(", "").replace(")", "").split(', ')))[0]
            z = tuple(map(float, csvreader[row][0].replace("(", "").replace(")", "").split(', ')))[2]
            if x == current_location[0] and z == current_location[1]:
                orientation_count = orientation_count + int(csvreader[row][4])
            else:
                orientations.append(orientation_count)
                orientation_count = 0
                current_location = [x, z]
                locations_x.append(x)
                locations_z.append(z)
        locations_x.append(x)
        locations_z.append(z)
        orientations.append(orientation_count)'''


print(len(validLocations))

'''newValidLocations = []
total = len(validLocations)
for num in range(9):
    newValidLocations.append(validLocations[round((total*num)/9)])
validLocations = newValidLocations'''

if 0 < len(validLocations) <= 10:
    fullTrajectoriesInWorkspace = []
    partialTrajectoriesInWorkspace = []
    for location in tqdm(validLocations, desc='Create Trajectories for Graph'):
        for trajectory in trajectories[trackedMarker]['rotated']:
            trajectoryVector = vectorbetweenpoints(trajectories[trackedMarker]['rotated'][0], trajectory)
            trajectoryInWorkspace = np.array(location)+np.array(trajectoryVector)
            partialTrajectoriesInWorkspace.append(trajectoryInWorkspace)
        fullTrajectoriesInWorkspace.append(partialTrajectoriesInWorkspace)

    # Create a new plot
    figure = plt.figure()
    axes = figure.add_subplot(projection='3d')
    workspace_mesh = mesh.Mesh.from_file(r'Iiwa 14 Worksace.stl')
    workspace_mesh.x -= 946
    workspace_mesh.y -= 946
    workspace_mesh.z -= 736
    poly_collection = mplot3d.art3d.Poly3DCollection(workspace_mesh.vectors, alpha=0.2)
    poly_collection.set_color((0.5, 0.5, 1))  # play with color
    axes.add_collection3d(poly_collection)

    for trajectory in fullTrajectoriesInWorkspace:
        x = []
        y = []
        z = []
        for location in trajectory:
            x.append(location[0])
            y.append(location[1])
            z.append(location[2])
        axes.scatter(x, y, z, c='m')
    axes.set_aspect('equal')
    axes.set_xlabel('x')
    axes.set_ylabel('y')
    axes.set_zlabel('z')
    # Show the plot to the screen
    plt.show()

elif len(validLocations) > 10:
    figure = plt.figure()
    axes = figure.add_subplot(projection='3d')
    workspace_mesh = mesh.Mesh.from_file(r'Iiwa 14 Worksace.stl')
    workspace_mesh.x -= 946
    workspace_mesh.y -= 946
    workspace_mesh.z -= 736
    poly_collection = mplot3d.art3d.Poly3DCollection(workspace_mesh.vectors, alpha=0.2)
    poly_collection.set_color((0.5, 0.5, 1))  # play with color
    axes.add_collection3d(poly_collection)
    '''convex_hull_id = validLocations.add_structure("convex_hull")
    convex_hull = validLocations.structures[convex_hull_id]
    validLocations.mesh = convex_hull.get_mesh()
    poly_collection = mplot3d.art3d.Poly3DCollection(validLocations.mesh.vectors, alpha=0.2)
    poly_collection.set_color((0.5, 0.5, 1))  # play with color
    axes.add_collection3d(poly_collection)'''

    '''cloud = pv.PolyData(validLocations)
    volume = cloud.delaunay_3d(alpha=2.0)
    mesh = volume.extract_geometry()
    axes.add_collection3d(mesh)'''

    x = []
    y = []
    z = []
    for location in validLocations:
        x.append(location[0])
        y.append(location[1])
        z.append(location[2])
    axes.scatter(x, y, z, c='m')
    axes.set_aspect('equal')
    axes.set_xlabel('x')
    axes.set_ylabel('y')
    axes.set_zlabel('z')
    # Show the plot to the screen
    plt.show()

else:
    print('No valid locations')






'''
x = []
y = []
z = []

for location in validLocations:
    x.append(location[0])
    y.append(location[1])
    z.append(location[2])

fig = go.Figure(data=[go.Mesh3d(x=x, y=y, z=z,
                   alphahull=5,
                   opacity=0.4,
                   color='cyan')])
fig.show()
'''

