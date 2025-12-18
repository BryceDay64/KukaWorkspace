import matplotlib.pyplot as plt
import csv

# TODO: Import CSV
#       count successful orientations at each location
#       plot



orientation_count = 0
locations_x = []
locations_z = []
orientations = []
with open('testing1761770384821.9443', 'w', newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    current_location = [csvreader[1][0][0], csvreader[1][0][2]]
    for row in range(1, len(csvreader)):
        if csvreader[row][0][0] == current_location[0] and csvreader[row][0][2] == current_location[1]:
            orientation_count = orientation_count + csvreader[row][4]
        else:
            orientations.append(orientation_count)
            current_location = [csvreader[row][0][0], csvreader[row][0][2]]
            locations_x.append(csvreader[row][0][0])
            locations_z.append(csvreader[row][0][2])

    orientations.append(orientation_count)


[float(i) for i in orientations]
plt.scatter(locations_x, locations_z, c=orientations, cmap='tab20b', vmin=0, vmax=500)  # 'YlOrRd' 'tab20c'
plt.colorbar()
plt.show()