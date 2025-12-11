import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import math
import logging
import csv
import time


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

orientation_samples = 1000
z_location_divisions = 168
x_location_divisions = 189

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

for location in location_vectors_centered:
    if np.linalg.norm(location) < workspace_inner_bound:
        continue
    else:
        angle = math.atan2(location[2], location[0])
        angle = angle + math.pi/2
        if angle < 0:
            angle = angle + 2*math.pi
        if math.pi/3 <= angle <= (5*math.pi)/3:
            if np.linalg.norm(location) <= workspace_major_outer_bound:
                workspace_locations.append(location)
        elif math.pi <= angle <= 2*math.pi:
            new_center = (location[0]+363.73, 0, location[2]+210)
            if np.linalg.norm(new_center) <= workspace_minor_outer_bound:
                workspace_locations.append(location)
        else:
            new_center = (location[0]-363.73, 0, location[2]+210)
            if np.linalg.norm(new_center) <= workspace_minor_outer_bound:
                workspace_locations.append(location)

half_workspace_locations = []
for location in workspace_locations:
    if location[0] > 0:
        fixed_location = (location[0]/1000,
                          location[1]/1000,
                          (location[2]+360)/1000)
        half_workspace_locations.append(fixed_location)

print(len(half_workspace_locations))

lbr = rtb.models.URDF.LBR()                  # instantiate robot model
print(lbr)

T = lbr.fkine(lbr.qz, end='tool0')
print(T)

with open('iiwa14'+str(time.time()*1000)+'.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(fields)

    possible_orientations = []
    location_count = 0
    start_time = int(time.time() * 1000)
    for location in half_workspace_locations:
        location_count += 1
        orientation_count = 0
        location_time_start = int(time.time() * 1000)
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

            if vector_num is len(approach_vectors)-1:
                csvwriter.writerows([[location, approach_vectors[vector_num], tuple(orientation_vectors[vector_num]),
                                      tuple(alt_orientation_vectors[vector_num]),
                                      sol[1], tuple(sol[0]), orientation_count]])
            else:
                csvwriter.writerows([[location, approach_vectors[vector_num], tuple(orientation_vectors[vector_num]),
                                      tuple(alt_orientation_vectors[vector_num]),
                                      sol[1], tuple(sol[0])]])

        possible_orientations.append(orientation_count)

        now_time = int(time.time()*1000)
        location_running = time_difference(location_time_start, now_time, False)
        total_running = time_difference(start_time, now_time, True)
        logging.info("Location: " + str(location_count) + "/" + str(len(half_workspace_locations)) +
                     "\nPossible Orientations Found: " + str(orientation_count) +
                     "\nTime at this location: " + str(location_running) +
                     "\nTotal time running: " + str(total_running) +
                     "\n"
                     )

logging.info('Finished :' + str(possible_orientations))

while True:
    continue
