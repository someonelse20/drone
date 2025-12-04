import matplotlib.pyplot as plt
import numpy as np

x = 0
y = 1
z = 2

def read_data(file): # Converts raw imu output into a three dementional numpy array of the gyrometer data.
    xaxis = []
    yaxis = []
    zaxis = []

    datafile = open("./data/" + file, "rt")
    rawdata = datafile.read().split("\n")

    for line in rawdata:
        if "accel" in line:
            nums = line.replace("accel:", "").split(",")
            xaxis.append(float(nums[x]) - -0.02425592)
            yaxis.append(float(nums[y]) - 0.00482671)
            zaxis.append(float(nums[z]) - -0.01078422 + 1)

            """
            xaxis.append(float(nums[x]))
            yaxis.append(float(nums[y]))
            zaxis.append(float(nums[z]) + 1)
            """

    return np.array([xaxis, yaxis, zaxis])

data = read_data('az-')

plt.plot(np.arange(len(data[0])), data[x])
plt.plot(np.arange(len(data[0])), data[y])
plt.plot(np.arange(len(data[0])), data[z])
#plt.ylim(-0.1, 0.1)
plt.show()

