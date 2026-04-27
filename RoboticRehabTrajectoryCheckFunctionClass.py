import numpy as np
from tqdm import tqdm
import csv
import math
import statistics

# FILE IMPORTS
def import_vicon_markers(file):
    trajectories = {}
    with open(file, newline='') as csvfile:
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
                x.append(float(row[3 * marker + 2]))
                y.append(float(row[3 * marker + 3]))
                z.append(float(row[3 * marker + 4]))
                full.append([float(row[3 * marker + 2]), float(row[3 * marker + 3]), float(row[3 * marker + 4])])
            new_name = markers[marker].split(':')[1]
            trajectories.update({new_name: {'x': x,
                                           'y': y,
                                           'z': z,
                                           'full': full}})
    return trajectories

def import_workspace_voxels(file, v_end_start_p, v_centroid_com_p, tolerance_angle):
    with open(file, newline='') as csvfile:
        csvreader = csv.reader(csvfile)
        csvreader = list(csvreader)
        csvreader = csvreader[1:-1]
        workspace_points = []
        z_cutoff = np.abs(v_end_start_p[2]) - 736
        for location in tqdm(csvreader, desc='Import Voxels'):
            if float(location[2]) > z_cutoff:
                if v_centroid_com_p[0] >= 0:
                    if np.abs(np.rad2deg(np.arctan2(float(location[1]), float(location[0])))) >= tolerance_angle:
                        workspace_points.append([float(location[0]), float(location[1]), float(location[2])])
                else:
                    if np.abs(np.rad2deg(np.arctan2(float(location[1]), float(location[0])))) <= tolerance_angle:
                        workspace_points.append([float(location[0]), float(location[1]), float(location[2])])
    return workspace_points

def import_workspace_pixels(file, v_centroid_com_p, tolerance_angle):
    with open(file, newline='') as csvfile:
        csvreader = csv.reader(csvfile)
        csvreader = list(csvreader)
        csvreader = csvreader[1:-1]
        workspace_points = []
        for location in tqdm(csvreader, desc='Import Pixels'):
            if v_centroid_com_p[0] >= 0:
                if np.abs(np.rad2deg(np.arctan2(float(location[1]), float(location[0])))) >= tolerance_angle:
                    workspace_points.append([float(location[0]), float(location[1])])
            else:
                if np.abs(np.rad2deg(np.arctan2(float(location[1]), float(location[0])))) <= tolerance_angle:
                    workspace_points.append([float(location[0]), float(location[1])])
    return workspace_points

def import_approach_keys(file):
    with open(file, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            string_approach_keys = row

    # noinspection PyTypeChecker
    x = list(map(float, string_approach_keys['x'].replace("[", "").replace("]", "").split(', ')))
    # noinspection PyTypeChecker
    z = list(map(float, string_approach_keys['z'].replace("[", "").replace("]", "").split(', ')))
    # noinspection PyTypeChecker
    a = list(map(float, string_approach_keys['a'].replace("[", "").replace("]", "")
                 .replace("(", "").replace(")", "").split(', ')))

    x = np.array(x) * 1000
    z = list((np.array(z) * 1000) - 360)

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

    approach_keys = {'x': x, 'z': z, 'a': a}
    return approach_keys

def import_boolean_solution_tensor(file):
    boolean_solution_tensor = []
    boolean_solution_matrix = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        csvreader = list(csvreader)
        for row in tqdm(csvreader, desc='Load Approach Tensor'):
            for item in row:
                boolean_list = list(map(int, item.replace("[", "").replace("]", "").split(', ')))
                boolean_solution_matrix.append(boolean_list)
            boolean_solution_tensor.append(boolean_solution_matrix)
            boolean_solution_matrix = []
    return boolean_solution_tensor

# CHECKS
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
            trajectory_vector = np.array(trajectory_point)-np.array(full_trajectory[0])
            trajectory_in_workspace = np.array(voxel) + np.array(trajectory_vector)
            if not check_workspace(trajectory_in_workspace):
                full_trajectory_bool = False
                break
        if full_trajectory_bool:
            new_valid_voxels.append(voxel)
    return new_valid_voxels

def check_approach(voxels, keys, boolean_tensor, approach_vector, trajectory_vector, description):
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
        a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys) - np.array(approach_vector),
                                         axis=1))).argmin()
        if a_index >= 500:
            a_index -= 500
        if boolean_tensor[x_index][z_index][a_index]:
            new_valid_voxels.append(voxel)
    return new_valid_voxels

def check_approach_quick(voxels, keys, boolean_tensor, approach_vector, trajectory_vector):
    new_valid_voxels = []
    for voxel in voxels:
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
        a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys) - np.array(approach_vector),
                                         axis=1))).argmin()
        if a_index >= 500:
            a_index -= 500
        if boolean_tensor[x_index][z_index][a_index]:
            new_valid_voxels.append(voxel)
    return new_valid_voxels

def check_approach_full(voxels, keys, boolean_tensor, full_approaches, full_trajectory, description):
    new_valid_voxels = []
    for voxel in tqdm(voxels, desc=description):
        full_trajectory_bool = True
        for approach_index in range(len(full_approaches)):
            traj_vector = (np.array(full_trajectory[approach_index])
                           - np.array(full_trajectory[0]))
            traj_location = np.array(voxel) + np.array(traj_vector)
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
                                             - np.array(full_approaches[approach_index]), axis=1))).argmin()
            if a_index >= 500:
                a_index -= 500
            if not boolean_tensor[x_index][z_index][a_index]:
                full_trajectory_bool = False
                break
        if full_trajectory_bool:
            new_valid_voxels.append(voxel)
    return new_valid_voxels

def check_approach_full_quick(voxels, keys, boolean_tensor, full_approaches, full_trajectory):
    new_valid_voxels = []
    for voxel in voxels:
        full_trajectory_bool = True
        for approach_index in range(len(full_approaches)):
            traj_vector = (np.array(full_trajectory[approach_index])
                           - np.array(full_trajectory[0]))
            traj_location = np.array(voxel) + np.array(traj_vector)
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
                                             - np.array(full_approaches[approach_index]), axis=1))).argmin()
            if a_index >= 500:
                a_index -= 500
            if not boolean_tensor[x_index][z_index][a_index]:
                full_trajectory_bool = False
                break
        if full_trajectory_bool:
            new_valid_voxels.append(voxel)
    return new_valid_voxels

def check_planar_workspace(point):
    workspace_major_outer_bound = 946  # mm
    projection = np.array([point[0], point[1]])
    r = np.linalg.norm(projection)
    if r < workspace_major_outer_bound:
        in_workspace = True
    else:
        in_workspace = False
    return in_workspace

def check_inner_bound_workspace(point):
    workspace_inner_bound = 280
    projection = np.array(point[0], point[1])
    r = np.linalg.norm(projection)
    if r > workspace_inner_bound:
        in_workspace = True
    else:
        in_workspace = False
    return in_workspace

def check_extending_location(pixels, trajectory_vector, description):
    new_valid_pixels = []
    for pixel in tqdm(pixels, desc=description):
        traj_location = np.array(pixel) + np.array([trajectory_vector[0], trajectory_vector[1]])
        if check_planar_workspace(traj_location):
            new_valid_pixels.append(pixel)
    return new_valid_pixels

def check_approach_get_index(voxels, keys, index_tensor, approach_vector, trajectory_vector, description):
    new_valid_voxels = []
    valid_voxel_index = []
    angles = []
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
        a_index = (np.abs(np.linalg.norm(np.array(rotated_approach_keys) - np.array(approach_vector),
                                         axis=1))).argmin()
        if a_index >= 500:
            a_index -= 500
        if index_tensor[x_index][z_index][a_index] != 0:
            valid_voxel_index.append(index_tensor[x_index][z_index][a_index])
            new_valid_voxels.append(voxel)
            angles.append(angle)
    return new_valid_voxels, valid_voxel_index, angles

# Trajectory calculations
def perform_vector_calculations(trajectories, location_marker, origin_marker, a_marker, b_marker):
    com = [statistics.fmean(trajectories['COM']['x']),
           statistics.fmean(trajectories['COM']['y']),
           statistics.fmean(trajectories['COM']['z'])]

    centroid_location = [statistics.fmean(trajectories[location_marker]['x']),
                         statistics.fmean(trajectories[location_marker]['y']),
                         statistics.fmean(trajectories[location_marker]['z'])]

    v_centroid_com = np.array(centroid_location)-np.array(com)
    ang_centroid_com_x = np.arctan2(v_centroid_com[1], v_centroid_com[0])

    translation_vector = np.array([-com[0], -com[1], 0])

    com_p = np.array(com)+translation_vector

    # Rotates all points in trajectory about com to align centroid with x
    trajectory_rotation_matrix_com = np.array([[np.cos(ang_centroid_com_x), -np.sin(ang_centroid_com_x), 0],
                                               [np.sin(ang_centroid_com_x), np.cos(ang_centroid_com_x), 0],
                                               [0, 0, 1]])

    for marker in trajectories:
        aligned_trajectory = []
        for trajectory in trajectories[marker]['full']:
            trajectory_translation = np.array(trajectory)+translation_vector
            trajectory_rotation = np.matmul(np.array(trajectory_translation), trajectory_rotation_matrix_com)
            aligned_trajectory.append(trajectory_rotation)
        trajectories[marker]['rotated'] = aligned_trajectory

    v_end_start_p = (np.array(trajectories[location_marker]['rotated'][-1])
                   - np.array(trajectories[location_marker]['rotated'][0]))
    mag_end_start_p = np.linalg.norm(v_end_start_p)

    largest_distance = 0
    largest_normal_index = int(round(len(trajectories[location_marker]['rotated']) / 2))
    for location_num in range(len(trajectories[location_marker]['rotated'])):
        v_start_traj = (np.array(trajectories[location_marker]['rotated'][0])
                        - np.array(trajectories[location_marker]['rotated'][location_num]))
        distance = ((np.linalg.norm(np.cross(v_end_start_p, v_start_traj))) / mag_end_start_p)
        if distance > largest_distance:
            largest_distance = distance
            largest_normal_index = location_num
    v_largest_normal_start_p = (np.array(trajectories[location_marker]['rotated'][largest_normal_index])
                                - np.array(trajectories[location_marker]['rotated'][0]))
    v_com_start_p = np.array(com_p) - np.array(trajectories[location_marker]['rotated'][0])
    centroid_location_p = np.matmul(centroid_location+translation_vector,trajectory_rotation_matrix_com)
    v_centroid_com_p = np.array(centroid_location_p)-np.array(com_p)

    approaches = []
    for index in range(len(trajectories[location_marker]['rotated'])):
        a_vector = (np.array(trajectories[a_marker]['rotated'][index])
                         - np.array(trajectories[origin_marker]['rotated'][index]))
        b_vector = (np.array(trajectories[b_marker]['rotated'][index])
                          - np.array(trajectories[origin_marker]['rotated'][index]))
        approach_vector = np.cross(np.array(a_vector), np.array(b_vector))
        approach_unit_vector = approach_vector / np.linalg.norm(approach_vector)
        approaches.append(approach_unit_vector)
    return v_end_start_p, v_largest_normal_start_p, v_com_start_p, v_centroid_com_p, com_p, approaches, largest_normal_index
