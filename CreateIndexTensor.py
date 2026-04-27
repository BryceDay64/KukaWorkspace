import csv
from tqdm import tqdm
import numpy as np

z_location_divisions = 168
x_location_divisions = 189
orientations = 500

index_list = []
index_matrix = []
with open('iiwa141762967540628.9177.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    csvreader = list(csvreader)
    z = []
    x = []
    a = []
    i = 0
    past_x = tuple(map(float, csvreader[1][0].replace("(", "").replace(")", "").split(', ')))[0]
    past_z = tuple(map(float, csvreader[1][0].replace("(", "").replace(")", "").split(', ')))[2]
    outer_formatter = []
    inner_formatter = []
    for row in tqdm(range(1, len(csvreader))):
        i += 1
        current_x = tuple(map(float, csvreader[row][0].replace("(", "").replace(")", "").split(', ')))[0]
        current_z = tuple(map(float, csvreader[row][0].replace("(", "").replace(")", "").split(', ')))[2]
        current_a = tuple(map(float, csvreader[row][1].replace("(", "").replace(")", "").split(', ')))
        if int(csvreader[row][4]) == 1:
            index_list.append(int(row))
        else:
            index_list.append(0)
        #add index list here

        if not current_z == past_z:
            inner_formatter.append(past_z)
            if not current_x == past_x:
                outer_formatter.append(inner_formatter)
                inner_formatter = []
                index_count = 0

        if i == 500:
            index_matrix.append(index_list)
            index_list = []
            i = 0
        if current_x not in x:
            x.append(current_x)
        if current_z not in z:
            z.append(current_z)
        if current_a not in a:
            a.append(current_a)
        past_x = current_x
        past_z = current_z

index_count = 0
index_solutions_tensor = np.zeros((len(x), len(z), len(a))).tolist()
for inner in range(len(outer_formatter)):
    for z_finder in outer_formatter[inner]:
        z_index = z.index(z_finder)
        index_solutions_tensor[inner][z_index] = index_matrix[index_count]
        index_count += 1

for matrix in range(len(index_solutions_tensor)):
    for vector in range(len(index_solutions_tensor[matrix])):
        if all([v == 0 for v in index_solutions_tensor[matrix][vector]]):
            index_solutions_tensor[matrix][vector] = [0]*500

if len(index_solutions_tensor) != len(x):
    print(len(index_solutions_tensor))
    print('outer incorrect')
for outer in index_solutions_tensor:
    if len(outer) != len(z):
        print(len(outer))
        print('middle incorrect')
    for middle in outer:
        if len(middle) != len(a):
            '''if len(middle) != 0:'''
            print(len(middle))
            print('inner incorrect')


'''print(len(x))
print(len(z))
print(len(a))

orientation_keys = {'x': x, 'z': z, 'a': a}

with open("Iiwa14OrientationKeys.csv", "w", newline="") as f:
    w = csv.DictWriter(f, orientation_keys.keys())
    w.writeheader()
    w.writerow(orientation_keys)'''

with open("index solution tensor.csv", "w", newline="") as f:
    w = csv.writer(f)
    for row in index_solutions_tensor:
        w.writerow(row)
