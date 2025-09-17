import matplotlib.pyplot as plt

def read_file(file):
    xs = []
    ys = []
    zs = []


    datafile = open("./data/" + file, "rt")
    rawdata = datafile.read().split("\n")

    for line in rawdata:
        if "mag" in line:
            nums = line.replace("mag:", "").split(",")
            xs.append(float(nums[0]))
            ys.append(float(nums[1]))
            zs.append(float(nums[2]))

    return [xs, ys, zs]

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

files = ["m1", "m2", "m3"]

for file in files:
    points = read_file(file)
    ax.scatter(points[0], points[1], points[2])

ax.quiver(0, 0, 0, 0.5, 0, 0)
ax.quiver(0, 0, 0, 0, 0.5, 0)
ax.quiver(0, 0, 0, 0, 0, 0.5)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
