import numpy as np
import math
from tqdm import tqdm
import csv

class CheckFullWorkspace:
    def __init__(self):
        def check_workspace(point):
            """Given a point, this function will check if it is in the Iiwa 14's workspace and will return a bool."""
            workspace_inner_bound = 280  # mm
            workspace_major_outer_bound = 946  # mm
            workspace_minor_outer_bound = 526  # mm
            r = np.linalg.norm(point)
            if r < workspace_inner_bound:
                in_workspace = False
            else:
                azimuth = np.arccos(point[2] / r)
                inclination = np.arctan2(point[1], point[0])
                if 2 * math.pi / 3 >= azimuth:
                    if r <= workspace_major_outer_bound:
                        in_workspace = True
                    else:
                        in_workspace = False
                else:
                    new_center = (363.73 * np.cos(inclination) + point[0],
                                  363.73 * np.sin(inclination) + point[1],
                                  210 + point[2])
                    if np.linalg.norm(new_center) <= workspace_minor_outer_bound:
                        in_workspace = True
                    else:
                        in_workspace = False
            return in_workspace

class CheckPlanarWorkspace:
    def __init__(self):
        def check_workspace(point):
            workspace_major_outer_bound = 946  # mm
            projection = np.array(point[0],point[1])
            r = np.linalg.norm(projection)
            if r < workspace_major_outer_bound:
                in_workspace = True
            else:
                in_workspace = False
            return in_workspace

class ImportFiles:
    def __init__(self):
        def import_markers(self, file):
            self.trajectories = {}
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
                        full.append(
                            [float(row[3 * marker + 2]), float(row[3 * marker + 3]), float(row[3 * marker + 4])])
                    newName = markers[marker].split(':')[1]
                    self.trajectories.update({newName: {'x': x,
                                                   'y': y,
                                                   'z': z,
                                                   'full': full}})
            return self.trajectories
'''
class Iiwa14Flat:
    def __init__(self, wsc):
        def calculate_vectors(trajectory):
            v_end_start = (np.array(trajectories[trackedMarker]['full'][-1])
                           - np.array(trajectories[trackedMarker]['full'][0]))
            mag_end_start = np.linalg.norm(v_end_start)
            h_end_start = np.abs(v_end_start[2])
            z_cutoff = h_end_start - 736

            largestDistance = 0
            largestNormalIndex = int(round(len(trajectories[trackedMarker]['full']) / 2))
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

            v_centroid_start = np.array(centroid_location) - np.array(trajectories[trackedMarker]['full'][0])

            v_com_start = np.array(COM) - np.array(trajectories[trackedMarker]['full'][0])
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
                    rotAboutCOM = np.matmul(np.array(trajectory) - np.array(COM), trajectory_rotation_matrix_COM)
                    rotAboutOrg = np.matmul(rotAboutCOM + np.array(COM), trajectory_rotation_matrix_org)
                    rotToFix = np.matmul(rotAboutOrg, trajectory_rotation_matrix_fix)
                    alignedTrajectory.append(rotToFix)
                trajectories[marker]['rotated'] = alignedTrajectory
            v_end_start_p = np.array(trajectories[trackedMarker]['rotated'][-1]) - np.array(
                trajectories[trackedMarker]['rotated'][0])
            v_largestNormal_start_p = (np.array(trajectories[trackedMarker]['rotated'][largestNormalIndex])
                                       - np.array(trajectories[trackedMarker]['rotated'][0]))
            v_COM_start_p = np.array(COM_p) - np.array(trajectories[trackedMarker]['rotated'][0])
            rot_incenter_COM = np.matmul(v_centroid_com, trajectory_rotation_matrix_COM)
            rot_incenter_org = np.matmul(rot_incenter_COM + np.array(COM_p), trajectory_rotation_matrix_org) - COM_p
            v_incenter_com_p = np.matmul(rot_incenter_org + np.array(COM_p), trajectory_rotation_matrix_fix) - COM_p

            orientations = []
            for index in range(len(trajectories[trackedMarker]['rotated'])):
                medialVector = (np.array(trajectories[medialMarker]['rotated'][index])
                                - np.array(trajectories[originMarker]['rotated'][index]))
                lateralVector = (np.array(trajectories[lateralMarker]['rotated'][index])
                                 - np.array(trajectories[originMarker]['rotated'][index]))
                if palmarOrientation and rightHand:
                    orientationVector = np.cross(np.array(medialVector), np.array(lateralVector))
                    orientationUnitVector = orientationVector / np.linalg.norm(orientationVector)
                    orientations.append(orientationUnitVector)

            rotated_x = []
            rotated_y = []
            rotated_z = []
            for trajectory in trajectories[trackedMarker]['rotated']:
                rotated_x.append(trajectory[0])
                rotated_y.append(trajectory[1])
                rotated_z.append(trajectory[2] - COM_p[2])



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
                    trajectory_vector = np.array(trajectory_point) - np.array(trajectories[trackedMarker]['rotated'][0])
                    trajectory_in_workspace = np.array(voxel) + np.array(trajectory_vector)
                    if not check_workspace(trajectory_in_workspace):
                        full_trajectory_bool = False
                        break
                if full_trajectory_bool:
                    new_valid_voxels.append(voxel)
            return new_valid_voxels

        def check_orientation(voxels, keys, boolean_tensor, orientation_vector, trajectory_vector, description):
            new_valid_voxels = []
            for voxel in tqdm(voxels, desc=description):
                traj_location = np.array(voxel) + np.array(trajectory_vector)
                z_index = (np.abs(np.array(keys['z']) - traj_location[2])).argmin()
                x_index = (np.abs(
                    np.array(keys['x']) - np.linalg.norm(np.array([traj_location[0], traj_location[1]])))).argmin()
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
                    new_valid_voxels.append(voxel)
            return new_valid_voxels

        def check_orientation_full(voxels, keys, boolean_tensor, full_orientations, full_trajectory, description):
            new_valid_voxels = []
            for voxel in tqdm(voxels, desc=description):
                full_trajectory_bool = True
                for orientation_index in range(len(full_orientations)):
                    traj_location = np.array(voxel) + np.array(full_trajectory[orientation_index])
                    z_index = (np.abs(np.array(keys['z']) - traj_location[2])).argmin()
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
                                                     - np.array(full_orientations[orientation_index]),
                                                     axis=1))).argmin()
                    if a_index >= 500:
                        a_index -= 500
                    if not boolean_tensor[x_index][z_index][a_index]:
                        full_trajectory_bool = False
                        break
                if full_trajectory_bool:
                    new_valid_voxels.append(voxel)
            return new_valid_voxels


class Iiwa14extended:
     def __init__(self):

        def check_location():
            pass

class Iiwa14angled:
    def __init__(self):
        pass

'''