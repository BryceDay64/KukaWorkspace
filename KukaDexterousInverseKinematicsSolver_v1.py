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


def time_difference(first_time, second_time, need_hours):
    time_diff = second_time - first_time
    difference_seconds = round(((time_diff / 1000) % 60) % 10, 3)
    difference_ten_seconds = math.floor(((time_diff / 1000) % 60) / 10)
    difference_minutes = math.floor((time_diff / 60000 % 100) % 10)
    difference_ten_minutes = math.floor(((time_diff / 60000) % 100) / 10)
    if need_hours:
        difference_hours = math.floor(time_diff / 3600000 % 100)
        start_to_end_difference_formatted = (str(difference_hours) + ":" +
                                             str(difference_ten_minutes) +
                                             str(difference_minutes) + ":" +
                                             str(difference_ten_seconds) +
                                             str(difference_seconds))
    else:
        start_to_end_difference_formatted = (str(difference_ten_minutes) +
                                             str(difference_minutes) + ":" +
                                             str(difference_ten_seconds) +
                                             str(difference_seconds))
    return start_to_end_difference_formatted


logging.basicConfig(
     filename="DexterousWorkspaceLog.log",
     level=logging.INFO
)

fields = ['position', 'approach', 'orientation',
          'alt orientation', 'solution boolean', 'solution', 'total possible orientations']

orientation_samples = 8000
z_location_divisions = 1683
x_location_divisions = 1893

workspace_width = 1892  # mm
workspace_height = 1682  # mm
workspace_inner_bound = 280  # mm
workspace_major_outer_bound = 946  # mm
workspace_minor_outer_bound = 526  # mm

location_vectors = []
for x_num in range(x_location_divisions):
    x_location = workspace_width*(x_num/(x_location_divisions-1))
    for z_num in range(z_location_divisions):
        z_location = workspace_height*(z_num/(z_location_divisions-1))
        location_vectors.append((x_location, 0, z_location))

location_vectors_centered = []
for location in location_vectors:
    location_vectors_centered.append((location[0]-(workspace_width/2), 0, location[2]-736))

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

workspace_locations = []
workspace_shape = []
# TODO: check inner pit
for location in location_vectors_centered:
    if np.linalg.norm(location) < workspace_inner_bound:
        workspace_shape.append(-50)
    else:
        angle = math.atan2(location[2], location[0])
        angle = angle + math.pi/2
        if angle < 0:
            angle = angle + 2*math.pi
        if math.pi/3 <= angle <= (5*math.pi)/3:
            if np.linalg.norm(location) <= workspace_major_outer_bound:
                workspace_locations.append(location)
                workspace_shape.append(0)
            else:
                workspace_shape.append(-50)
        elif math.pi <= angle <= 2*math.pi:
            new_center = (location[0]+363.73, 0, location[2]+210)
            if np.linalg.norm(new_center) <= workspace_minor_outer_bound:
                workspace_locations.append(location)
                workspace_shape.append(0)
            else:
                workspace_shape.append(-50)
        else:
            new_center = (location[0]-363.73, 0, location[2]+210)
            if np.linalg.norm(new_center) <= workspace_minor_outer_bound:
                workspace_locations.append(location)
                workspace_shape.append(0)
            else:
                workspace_shape.append(-50)

'''
x_points = []
z_points = []
for location in workspace_locations:
    x_points.append(location[0])
    z_points.append(location[2])

plt.plot(x_points, z_points, 'o')
plt.show()
'''

half_workspace_locations = []
for location in workspace_locations:
    if location[0] > 0:
        fixed_location = (location[0]/1000,
                          location[1]/1000,
                          (location[2]+360)/1000)
        half_workspace_locations.append(fixed_location)

x_points = []
z_points = []
for location in half_workspace_locations:
    x_points.append(location[0])
    z_points.append(location[2])

plt.plot(x_points, z_points, 'o')
plt.show()

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
x_points = []
y_points = []
z_points = []
for vector in approach_vectors:
    x_points.append(vector[0])
    y_points.append(vector[1])
    z_points.append(vector[2])
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.scatter(x_points, y_points, z_points)
ax.set_aspect('equal')
plt.show()

lbr = rtb.models.URDF.LBR()                  # instantiate robot model
print(lbr)

T = lbr.fkine(lbr.qz, end='tool0')
print(T)

possible_orientations = []
location_count = 0
for location in half_workspace_locations:
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
        logging.info("Location: " + str(location_count) + "/" + str(len(half_workspace_locations)) +
                     ", Orientation: " + str(vector_num) + "/" + str(len(approach_vectors)) +
                     ", Possible Orientations Found: " + str(possible_orientations))
        print("Location: " + str(location_count) + "/" + str(len(half_workspace_locations)) +
              ", Orientation: " + str(vector_num) + "/" + str(len(approach_vectors)) +
              ", Possible Orientations Found: " + str(possible_orientations))
    possible_orientations.append(orientation_count)

print('Finished :' + str(possible_orientations))


possible_orientations = [3, 16, 27, 23, 26, 24, 21, 16, 9, 2, 4, 11, 19, 27, 32, 37, 42, 47, 46, 45, 44, 42, 41, 39, 34, 26, 20, 13, 7, 2, 8, 15, 24, 23, 27, 25, 20, 17, 9, 4, 5, 13, 19, 27, 33, 39, 43, 47, 46, 45, 44, 42, 41, 37, 33, 26, 20, 13, 7, 7, 13, 19, 25, 28, 27, 27, 23, 18, 13, 6, 7, 15, 22, 27, 33, 38, 43, 48, 46, 45, 43, 43, 40, 37, 32, 24, 17, 12, 6, 4, 9, 14, 22, 27, 32, 29, 31, 25, 22, 15, 9, 2, 3, 11, 17, 24, 29, 35, 41, 45, 47, 46, 44, 43, 43, 39, 36, 31, 23, 16, 11, 5, 6, 13, 19, 25, 31, 35, 35, 31, 25, 26, 20, 13, 7, 7, 15, 20, 27, 32, 35, 42, 47, 47, 45, 45, 43, 42, 39, 35, 29, 22, 16, 10, 3, 2, 8, 14, 20, 26, 35, 38, 37, 34, 33, 26, 23, 18, 13, 7, 0, 7, 14, 17, 25, 28, 35, 39, 44, 46, 46, 46, 45, 42, 40, 39, 34, 28, 20, 14, 8, 2, 4, 11, 17, 24, 29, 35, 40, 39, 41, 36, 32, 26, 22, 18, 14, 8, 2, 4, 9, 13, 18, 25, 29, 33, 39, 43, 46, 46, 46, 46, 44, 40, 40, 36, 32, 24, 20, 13, 6, 1, 6, 12, 18, 24, 32, 35, 40, 43, 43, 39, 36, 32, 28, 22, 20, 15, 12, 7, 4, 2, 1, 1, 3, 4, 7, 13, 15, 22, 24, 29, 32, 37, 40, 45, 47, 46, 45, 45, 42, 40, 39, 34, 29, 21, 15, 11, 5, 6, 12, 18, 26, 33, 36, 40, 42, 44, 43, 41, 36, 33, 30, 26, 21, 18, 17, 15, 12, 12, 12, 12, 14, 16, 19, 22, 28, 31, 34, 36, 40, 45, 47, 45, 45, 45, 44, 41, 39, 38, 32, 26, 20, 13, 7, 1, 6, 11, 17, 25, 31, 37, 39, 43, 43, 43, 41, 39, 37, 33, 31, 29, 27, 24, 22, 21, 20, 20, 22, 22, 23, 26, 29, 32, 36, 39, 40, 43, 47, 47, 45, 45, 44, 43, 40, 39, 33, 28, 22, 16, 12, 6, 7, 12, 18, 24, 31, 38, 39, 42, 43, 43, 45, 43, 41, 38, 38, 33, 34, 30, 30, 29, 28, 29, 30, 31, 31, 31, 35, 38, 41, 42, 45, 47, 47, 47, 44, 44, 43, 40, 40, 36, 32, 24, 20, 14, 7, 2, 4, 10, 16, 23, 30, 35, 38, 40, 42, 43, 44, 46, 46, 43, 43, 40, 39, 35, 35, 34, 34, 36, 36, 37, 38, 38, 40, 44, 46, 48, 47, 47, 47, 43, 43, 43, 42, 40, 37, 33, 26, 21, 15, 9, 4, 3, 9, 15, 20, 28, 33, 37, 39, 40, 40, 42, 45, 45, 45, 45, 45, 43, 42, 41, 40, 40, 42, 42, 42, 44, 44, 45, 47, 47, 46, 46, 45, 45, 43, 43, 42, 40, 37, 34, 28, 23, 17, 12, 5, 1, 1, 6, 12, 18, 25, 33, 35, 38, 40, 40, 41, 44, 44, 44, 44, 47, 47, 48, 47, 46, 45, 45, 45, 46, 47, 47, 47, 47, 47, 45, 45, 45, 44, 43, 41, 41, 37, 34, 28, 23, 18, 13, 8, 3, 4, 9, 16, 21, 27, 33, 37, 39, 41, 41, 42, 44, 44, 45, 45, 45, 46, 48, 48, 47, 46, 47, 47, 47, 47, 46, 46, 44, 44, 44, 43, 42, 41, 40, 37, 33, 27, 23, 17, 13, 8, 3, 2, 6, 12, 17, 22, 28, 34, 36, 38, 40, 42, 42, 44, 45, 45, 44, 45, 45, 45, 45, 46, 46, 46, 47, 46, 45, 44, 44, 44, 43, 41, 40, 39, 35, 33, 27, 22, 17, 12, 9, 3, 4, 8, 13, 19, 22, 27, 32, 36, 37, 38, 39, 41, 42, 43, 43, 44, 44, 44, 45, 45, 45, 45, 43, 42, 41, 42, 42, 40, 38, 37, 37, 34, 28, 26, 20, 17, 14, 9, 4, 0, 5, 7, 13, 17, 22, 26, 30, 34, 37, 38, 38, 39, 41, 42, 42, 43, 43, 43, 43, 41, 42, 42, 41, 41, 41, 38, 37, 37, 35, 31, 27, 22, 19, 15, 13, 7, 4, 0, 5, 8, 12, 17, 19, 22, 27, 31, 31, 36, 37, 39, 39, 39, 40, 41, 41, 41, 41, 40, 40, 39, 37, 37, 35, 33, 30, 26, 23, 20, 17, 13, 9, 7, 2, 4, 7, 10, 14, 17, 20, 22, 26, 29, 29, 33, 34, 36, 37, 36, 36, 36, 36, 35, 35, 33, 32, 30, 28, 26, 23, 20, 16, 15, 11, 7, 4, 1, 2, 4, 9, 10, 14, 15, 18, 20, 22, 23, 27, 27, 28, 30, 29, 29, 27, 27, 27, 27, 25, 23, 19, 18, 16, 13, 10, 8, 5, 2, 2, 4, 8, 10, 13, 13, 15, 18, 19, 19, 21, 21, 22, 21, 20, 19, 19, 18, 17, 17, 11, 11, 10, 7, 4, 2, 1, 2, 6, 6, 8, 10, 11, 12, 13, 14, 13, 13, 13, 13, 11, 10, 10, 10, 6, 5, 4, 1, 1, 1, 4, 5, 5, 6, 7, 7, 7, 7, 7, 6, 5, 4, 2, 1]
cut_matrix = np.array(workspace_shape).reshape(z_location_divisions, x_location_divisions)
cut_matrix = np.delete(cut_matrix, np.s_[0:int(x_location_divisions/2)], axis=0)
half_workspace_shape = np.ndarray.flatten(cut_matrix)

shape_count = 0
for location_num in range(len(half_workspace_shape)):
    if half_workspace_shape[location_num] == 0:
        half_workspace_shape[location_num] = possible_orientations[shape_count]
        shape_count += 1

color_matrix = np.flip(np.transpose(np.array(half_workspace_shape).reshape(int(x_location_divisions/2), z_location_divisions)),0)
plt.imshow(color_matrix, cmap='tab20b', vmin=0, vmax=50)
plt.colorbar()
plt.show()

[float(i) for i in possible_orientations]
x_points = []
z_points = []
for location in half_workspace_locations:
    x_points.append(location[0])
    z_points.append(location[2])

plt.scatter(x_points, z_points, c=possible_orientations, cmap='tab20b', vmin=0, vmax=50)  # 'YlOrRd' 'tab20c'
plt.colorbar()
plt.show()


'''n = len(colormaps)
fig, axs = plt.subplots(1, n, figsize=(n * 2 + 2, 3), layout='constrained', squeeze=False)
for [ax, cmap] in zip(axs.flat, colormaps):
    psm = ax.pcolormesh(data, cmap=cmap, rasterized=True, vmin=-50, vmax=50)
    fig.colorbar(psm, ax=ax)
plt.show()'''

'''# Tep = sm.SE3.Trans(0.3, 0, 0.36) * sm.SE3.OA([0, 1, 0], [-1, 0, 0])
# Tep = sm.SE3.Trans(0.3, 0, 0.36) * sm.SE3.OA([0, -1, 0], [1, 0, 0])
print(half_workspace_locations[300])
print(orientation_vectors[49])
print(approach_vectors[49])
Tep = (sm.SE3.Trans(half_workspace_locations[300][0],
                    half_workspace_locations[300][1],
                    half_workspace_locations[300][2])
       * sm.SE3.OA([orientation_vectors[49][0],
                    orientation_vectors[49][1],
                    orientation_vectors[49][2]],
                   [approach_vectors[49][0],
                    approach_vectors[49][1],
                    approach_vectors[49][2]]))

sol = lbr.ik_LM(Tep, joint_limits=True)

print(sol)
print(sol[0]*180/np.pi)


qt = rtb.jtraj(lbr.qr, sol[0], 50)
lbr.plot(qt.q, backend='pyplot')'''
