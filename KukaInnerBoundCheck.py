import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import math
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import logging
import csv


def fibonacci_sphere(samples=1000):

    points = []
    phi = math.pi * (math.sqrt(5.) - 1.)  # golden angle in radians

    for i in range(samples):
        y = 1 - (i / float(samples - 1)) * 2  # y goes from 1 to -1
        radius = math.sqrt(1 - y * y)  # radius at y

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius
        z = math.sin(theta) * radius

        points.append((x, y, z))

    return points


orientation_samples = 90*90*2
bound_samples = 180
bound_location = 284.52767879  # mm

angle_step = math.pi/bound_samples
angle = -math.pi/2
locations = []
while angle <= math.pi/2:
    x_location = bound_location*math.cos(angle)
    z_location = bound_location*math.sin(angle)
    locations.append((x_location, 0, z_location))
    angle = angle + angle_step

workspace_locations = []
for location in locations:
    if location[0] > 0:
        fixed_location = (location[0]/1000,
                          location[1]/1000,
                          (location[2]+360)/1000)
        workspace_locations.append(fixed_location)

x_points = []
z_points = []

approach_sphere = fibonacci_sphere(orientation_samples)
approach_vectors = []
for point in approach_sphere:
    if point[1] >= 0:
        approach_vectors.append(point)

orientation_vectors = []
for point_num in range(len(approach_vectors)):
    attempt = 1
    while True:
        cross_vector = np.cross(np.array(approach_vectors[point_num]),
                                np.array(approach_vectors[point_num-attempt]))
        if np.linalg.norm(cross_vector) != 0:
            orientation_vectors.append(np.array(cross_vector).tolist())
            break
        else:
            attempt += 1

alt_orientation_vectors = []
for point in orientation_vectors:
    alt_orientation_vectors.append(np.array(np.array(point)*-1).tolist())

x_points = []
z_points = []
for location in workspace_locations:
    x_points.append(location[0])
    z_points.append(location[2])
plt.gca().set_aspect('equal')
plt.plot(x_points, z_points, 'o')
plt.show()

lbr = rtb.models.URDF.LBR()                  # instantiate robot model
print(lbr)

possible_orientations = []
location_count = 0
for location in workspace_locations:
    location_count += 1
    orientation_count = 0
    for vector_num in range(len(approach_vectors)):
        Tep = (sm.SE3.Trans(location[0], location[1], location[2])
               * sm.SE3.OA([orientation_vectors[vector_num][0],
                           orientation_vectors[vector_num][1],
                           orientation_vectors[vector_num][2]],
                           [approach_vectors[vector_num][0],
                           approach_vectors[vector_num][1],
                           approach_vectors[vector_num][2]]))
        sol = lbr.ik_LM(Tep, joint_limits=True)
        if sol[1] == 1:
            orientation_count += 1
        else:
            Tep = (sm.SE3.Trans(location[0], location[1], location[2])
                   * sm.SE3.OA([alt_orientation_vectors[vector_num][0],
                               alt_orientation_vectors[vector_num][1],
                               alt_orientation_vectors[vector_num][2]],
                               [approach_vectors[vector_num][0],
                               approach_vectors[vector_num][1],
                               approach_vectors[vector_num][2]]))
            sol = lbr.ik_LM(Tep, joint_limits=True)
            if sol[1] == 1:
                orientation_count += 1
        logging.info("Location: " + str(location_count) + "/" + str(len(workspace_locations)) +
                     ", Orientation: " + str(vector_num) + "/" + str(len(approach_vectors)) +
                     ", Possible Orientations Found: " + str(possible_orientations))
        print("Location: " + str(location_count) + "/" + str(len(workspace_locations)) +
              ", Orientation: " + str(vector_num) + "/" + str(len(approach_vectors)) +
              ", Possible Orientations Found: " + str(possible_orientations))
    possible_orientations.append(orientation_count)

print('Finished :' + str(possible_orientations))
