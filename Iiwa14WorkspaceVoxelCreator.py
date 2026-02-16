import numpy as np
import math
import csv
import matplotlib.pyplot as plt


def checkworkspace(point):
    workspace_inner_bound = 280  # mm
    workspace_major_outer_bound = 946  # mm
    workspace_minor_outer_bound = 526  # mm
    r = np.linalg.norm(point)
    if r < workspace_inner_bound:
        in_workspace = False
    else:
        azimuth =np.arccos(point[2]/r)
        inclination = np.arctan2(point[1], point[0])
        '''angle = math.atan2(point[2], point[0])
        angle = angle + math.pi / 2'''
        if azimuth < 0:
            '''angle = angle + 2 * math.pi'''
            print('negative')
        if 2*math.pi / 3 >= azimuth:
            if r <= workspace_major_outer_bound:
                in_workspace = True
            else:
                in_workspace = False
        else:
            new_center = (-363.73*np.cos(inclination)+point[0],
                          -363.73*np.sin(inclination)+point[1],
                          210+point[2])
            if np.linalg.norm(new_center) <= workspace_minor_outer_bound:
                in_workspace = True
            else:
                in_workspace = False
        '''elif math.pi <= angle <= 2 * math.pi:
            new_center = (point[0] + 363.73, 0, point[2] + 210)
            if np.linalg.norm(new_center) <= workspace_minor_outer_bound:
                in_workspace = True
            else:
                in_workspace = False
        else:
            new_center = (point[0] - 363.73, 0, point[2] + 210)
            if np.linalg.norm(new_center) <= workspace_minor_outer_bound:
                in_workspace = True
            else:
                in_workspace = False'''
    return in_workspace


z_location_divisions = 168
x_location_divisions = 189

workspace_width = 1892  # mm
workspace_height = 1682  # mm


workspace_locations = []
workspace_shape = []

location_vectors = []
for x_num in range(x_location_divisions):
    x_location = workspace_width*(x_num/(x_location_divisions-1))
    for z_num in range(z_location_divisions):
        z_location = workspace_height*(z_num/(z_location_divisions-1))
        for y_num in range(x_location_divisions):
            y_location = workspace_width*(y_num/(x_location_divisions-1))
            location_vectors.append((x_location, y_location, z_location))

location_vectors_centered = []
for location in location_vectors:
    location_vectors_centered.append((location[0]-(workspace_width/2),
                                      location[1]-(workspace_width/2),
                                      location[2]-736))

workspacePoints = []
for location in location_vectors_centered:
    if checkworkspace(location):
        workspacePoints.append(location)
x = []
y = []
z = []
for location in workspacePoints:
    if -10 < location[1] < 10:
        x.append(location[0])
        y.append(location[1])
        z.append(location[2])

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(x, y, z)
ax.set_aspect('equal')
plt.show()

fields = ['x', 'y', 'z']
with open('iiwa14WorkspaceVoxels.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(fields)

    for location in workspacePoints:
        csvwriter.writerow([location[0], location[1], location[2]])

