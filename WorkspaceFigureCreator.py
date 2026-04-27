import matplotlib.pyplot as plt
import csv
from tqdm import tqdm

# TODO: Import CSV
#       count successful orientations at each location
#       plot



orientation_count = 0
locations_x = []
locations_z = []
orientations = []
with open('iiwa141762967540628.9177.csv', newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    csvreader = list(csvreader)
    current_location = [csvreader[1][0][0], csvreader[1][0][2]]
    for row in tqdm(range(1, len(csvreader))):
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
    orientations.append(orientation_count)

print(max(orientations))

[float(i) for i in orientations]
fig, ax = plt.subplots()
im = ax.scatter(locations_x, locations_z, s=1**2, c=orientations, cmap='tab20b', vmin=0, vmax=500)  # 'tab20b' 'YlOrRd' 'tab20c'
cbar = fig.colorbar(im, ax=ax)
plt.xlabel('x (m)')
plt.ylabel('z (m)')
cbar.set_label("Number of valid approaches")
plt.show()