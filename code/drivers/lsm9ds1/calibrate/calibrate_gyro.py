import numpy as np
from numpy import linalg as la

x = 0 
y = 1 
z = 2

axes = ["x", "y", "z"]

REFERENCE_ANGULAR_VELOCITY = 200 # Reference angular velocity in degrees per second.

def read_data(file): # Converts raw imu output into a three dementional numpy array of the gyrometer data.
    xaxis = []
    yaxis = []
    zaxis = []

    datafile = open("./data/" + file, "rt")
    rawdata = datafile.read().split("\n")

    for line in rawdata:
        if "gyro" in line:
            nums = line.replace("gyro:", "").split(",")
            xaxis.append(float(nums[x]))
            yaxis.append(float(nums[y]))
            zaxis.append(float(nums[z]))

    return np.array([xaxis, yaxis, zaxis])

def data_average(file):
    dataset = read_data("g" + file)
    return np.array([np.average(dataset[x]), np.average(dataset[y]), np.average(dataset[z])])

def bias_calibration(): # Calculates the mean output of the gyrometer while it is staionary.
    return data_average("b")

def sensitivity_calibration(): # Calculates the average magnitude of each axis when exposed to a reference angular velocity.
    sensitivity = np.array([], dtype=float)

    for i in range(3):
        avg_plus = data_average(axes[i] + "+")
        avg_minus = data_average(axes[i] + "-")
        sensitivity = np.append(sensitivity, (np.abs(avg_plus[i]) + np.abs(avg_minus[i])) / (2 * REFERENCE_ANGULAR_VELOCITY))

    return sensitivity

def alignment_calibration(bias, sensitivity): # Calculatges the alignment of the gyrometer as a rotation matrix. DOES NOT WORK
    alignment = np.array([[0,0,0],[0,0,0],[0,0,0]])#, dtype=float)
    sensitivity_matrix = np.array([[1/sensitivity[x], 0, 0], [0, 1/sensitivity[y], 0], [0, 0, 1/sensitivity[z]]])

    for row in range(3):
        averages = data_average(axes[row] + "+")
        #print(la.norm(np.dot(sensitivity_matrix, np.subtract(averages, bias))))
        for column in range(3):
            alignment[column, row] = la.norm(np.dot(sensitivity_matrix, (averages[column] - bias[column])))

    return alignment

def test_calibration(bias, sensitivity, alignment):
    sensitivity_matrix = np.array([[1/sensitivity[x], 0, 0], [0, 1/sensitivity[y], 0], [0, 0, 1/sensitivity[z]]])

    not_moving = data_average("b")
    moving = data_average("x+")

    not_moving_value = alignment * sensitivity_matrix.transpose() * (np.array([[not_moving[x]], [not_moving[y]], [not_moving[z]]]) - np.array([[bias[x]], [bias[y]], [bias[z]]]))
    moving_value = np.dot(alignment, np.dot(sensitivity_matrix.transpose(), (np.array([[moving[x] - bias[x]], [moving[y] - bias[y]], [moving[z] - bias[z]]]))))
    test_value = np.dot(sensitivity_matrix, (np.array([[moving[x] - bias[x]], [moving[y] - bias[y]], [moving[z] - bias[z]]])))

    print(not_moving_value)
    print(moving_value)
    #print(test_value)

bias = bias_calibration()
sensitivity = sensitivity_calibration()
alignment = alignment_calibration(bias, sensitivity)
#print(bias)
#print(sensitivity)
#print(alignment)
test_calibration(bias, sensitivity, np.identity(3))

