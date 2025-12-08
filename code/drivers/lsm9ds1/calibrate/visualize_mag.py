import matplotlib.pyplot as plt
import numpy as np

import calibrate_mag

def read_file(file):
    points = np.array([])

    datafile = open("./data/" + file, "rt")
    rawdata = datafile.read().split("\n")

    for line in rawdata:
        if "mag" in line:
            string = line.replace("mag:", "").split(",")
            nums = np.zeros(3)
            for i in range(3):
                nums[i] = float(string[i])

            if points.size == 0:
                points = np.array([nums])
            else:
                points = np.append(points, [nums], axis=0)

    return points 

def calibrate(points, soft_iorn, hard_iorn):
    return_list = np.array([])

    for i in range(len(points)):
        if return_list.size == 0:
            return_list = np.array([np.dot(soft_iorn, points[i]) - hard_iorn])
        else:
            return_list = np.append(return_list, [np.dot(soft_iorn, points[i]) - hard_iorn], axis=0)

    return return_list

def scatter_points(points, set_color=''):
    xs = []
    ys = []
    zs = []

    for point in points:
        xs.append(point[0])
        ys.append(point[1])
        zs.append(point[2])

    ax.scatter(xs, ys, zs, color=set_color)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

points = read_file("m1")
points = np.append(points, read_file("m2"), axis=0)
points = np.append(points, read_file("m3"), axis=0)
points = np.append(points, read_file("m4"), axis=0)

#scatter_points(points, 'orange')

soft, hard = calibrate_mag.ellipsoid_calibration()

calibrated = calibrate(points, soft, hard)
scatter_points(points, 'gray')

xpoints = read_file("gx-")
ypoints = read_file("gy-")
zpoints = read_file("gz-")
scatter_points(xpoints, 'red')
scatter_points(ypoints, 'lightgreen')
scatter_points(zpoints, 'blue')

circle = read_file("mx+")
scatter_points(circle, 'orange')

ax.quiver(0, 0, 0, 0.5, 0, 0, color='red')
ax.quiver(0, 0, 0, 0, 0.5, 0, color='lightgreen')
ax.quiver(0, 0, 0, 0, 0, 0.5, color='blue')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# reference sphere
radius = 0.520411
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = radius * np.outer(np.cos(u), np.sin(v))
y = radius * np.outer(np.sin(u), np.sin(v))
z = radius * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot the surface
#ax.plot_surface(x, y, z, alpha=0.2)

# Set an equal aspect ratio
ax.set_aspect('equal')

plt.show()
