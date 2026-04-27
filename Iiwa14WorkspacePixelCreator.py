import numpy as np
import math
import csv
import matplotlib.pyplot as plt
import RoboticRehabTrajectoryCheckFunctionClass as rrc

workspace_width = 1892  # mm
x_location_divisions = 189

workspace_locations = []
workspace_shape = []

location_vectors = []
for x_num in range(x_location_divisions):
    x_location = workspace_width*(x_num/(x_location_divisions-1))
    for y_num in range(x_location_divisions):
        y_location = workspace_width*(y_num/(x_location_divisions-1))
        location_vectors.append((x_location, y_location))

location_vectors_centered = []
for location in location_vectors:
    location_vectors_centered.append((location[0]-(workspace_width/2),
                                      location[1]-(workspace_width/2)))

workspace_points = []
for location in location_vectors_centered:
    if rrc.check_planar_workspace(location):
        workspace_points.append(location)

x = []
y = []
for location in workspace_points:
    x.append(location[0])
    y.append(location[1])

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(x, y)
ax.set_aspect('equal')
plt.show()

fields = ['x', 'y']
with open('iiwa14WorkspacePixels.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(fields)

    for location in workspace_points:
        csvwriter.writerow([location[0], location[1]])
