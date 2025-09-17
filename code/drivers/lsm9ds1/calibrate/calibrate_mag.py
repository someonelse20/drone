from matplotlib.colors import NoNorm
import numpy as np
from numpy import linalg as la

x = 0 
y = 1 
z = 2

axes = ["x", "y", "z"]

MAGNETIC_FIELD = 0.520411

def read_data(file): # Converts raw imu output into a three dementional numpy array of the gyrometer data.
    data = []

    datafile = open("./data/" + file, "rt")
    rawdata = datafile.read().split("\n")

    for line in rawdata:
        if "mag" in line:
            nums = line.replace("mag:", "").split(",")
            dataline = []
            for num in nums:
                dataline.append(float(num))
            data.append(dataline)

    return data

def offset_calibration():
    M = np.array([])
    v = np.array([])
    for data in read_data("m1"):
        if M.size == 0:
            M = np.array([[-2 * data[x], 1, -2 * data[y], 1, -2 * data[z]]])
            v = np.array([[MAGNETIC_FIELD ** 2 - data[x] ** 2 - data[y] ** 2 - data[z] ** 2]])
        else:
            M = np.append(M, [[-2 * data[x], 1, -2 * data[y], 1, -2 * data[z]]], axis=0)
            v = np.append(v, [[MAGNETIC_FIELD ** 2 - data[x] ** 2 - data[y] ** 2 - data[z] ** 2]], axis=0)

    #h = np.dot(np.dot(la.inv(np.dot(M.transpose(), M)), M.transpose()), v)
    h = np.dot(la.pinv(M), v)

    print(np.allclose(np.dot(np.dot(M, la.pinv(M)),v), v))

    #print(h)

    #print(v)
    #print(np.dot(M, h))

    return np.array([h[0], h[2], h[4]])

offset = offset_calibration()
#print(offset)

