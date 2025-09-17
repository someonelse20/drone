import matplotlib.pyplot as plt
import numpy as np

def read_file(file):
    xs = []
    ys = []
    zs = []


    datafile = open("./data/" + file, "rt")
    rawdata = datafile.read().split("\n")

    for line in rawdata:
        if "accel" in line:
            nums = line.replace("accel:", "").split(",")
            xs.append(float(nums[0]))
            ys.append(float(nums[1]))
            zs.append(float(nums[2]))

    return np.array([xs, ys, zs])

def data_average(file):
    dataset = read_file(file)
    return np.array([np.average(dataset[0]), np.average(dataset[1]), np.average(dataset[2])])

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

files = ["ax+", "ay+", "az+"]

for file in files:
    points = data_average(file)
    ax.quiver(0, 0, 0, points[0], points[1], points[2], length=1)

ax.quiver(0, 0, 0, 1, 0, 0, color = 'red')
ax.quiver(0, 0, 0, 0, 1, 0, color = 'red')
ax.quiver(0, 0, 0, 0, 0, 1, color = 'red')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
