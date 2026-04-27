import RoboticRehabTrajectoryCheckFunctionClass as rrc
import numpy as np
import csv
from tqdm import tqdm

approach = np.array([1, 0, 0])

approachKeys = rrc.import_approach_keys('iiwa14ApproachKeys.csv')
index_solution_tensor = rrc.import_boolean_solution_tensor('index solution tensor.csv')

workspacePoints = rrc.import_workspace_voxels('iiwa14WorkspaceVoxels.csv', np.array([0, 0, 0]), np.array([-1, 0, 0]), 90)

validLocations, validIndexes, angles = rrc.check_approach_get_index(workspacePoints, approachKeys, index_solution_tensor, approach,
                                        np.array([0, 0, 0]),'Checking locations with forward facing orientation')

joints = []
with open('iiwa141762967540628.9177.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    csvreader = list(csvreader)
    for index in tqdm(validIndexes, desc="changing index to joints"):
        joints.append(list(map(float, csvreader[index][5].replace("(", "").replace(")", "").split(', '))))

print(joints[0])

for num in range(len(joints)):
    joints[num][0] = joints[num][0]+angles[num]

print(joints[0])

location_dict = {}
for num in tqdm(range(len(validLocations)), desc="Creating Dictionary of locations"):
    if str(validLocations[num][0]) in location_dict:
        location_dict[str(validLocations[num][0])].append(([validLocations[num][1], validLocations[num][2]+340], joints[num]))
    else:
        location_dict[str(validLocations[num][0])] = [([validLocations[num][1], validLocations[num][2]+340], joints[num])]

x_keys = list(location_dict.keys())

with open("Cartesian to Joint Balance.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(x_keys)
    for key in location_dict:
        w.writerow(location_dict[key])



