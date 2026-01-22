import numpy as np
from numpy import linalg as la

import json

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

def bias_calibration(file="az-"): # Calculates the mean output of the gyrometer while it is staionary.
    dataset = read_data(file) 
    return np.array([np.average(dataset[x]), np.average(dataset[y]), np.average(dataset[z])])

def sensitivity_calibration(): # Calculates the average magnitude of each axis when exposed to a reference angular velocity.
    sensitivity = np.array([], dtype=float)

    for i in range(3):
        avg_plus = data_average(axes[i] + "+")
        avg_minus = data_average(axes[i] + "-")
        sensitivity = np.append(sensitivity, (np.abs(avg_plus[i]) + np.abs(avg_minus[i])) / (2 * REFERENCE_ANGULAR_VELOCITY))

    return sensitivity

def alignment_calibration(bias, sensitivity): # Calculatges the alignment of the gyrometer as a rotation matrix. FIX BY NORMALIZING ALIGNMENT, PROBABLY BY NORMALIZING EACH VECTOR.
    alignment = np.array([[0,0,0],[0,0,0],[0,0,0]])#, dtype=float)
    sensitivity_matrix = np.array([[1/sensitivity[x], 0, 0], [0, 1/sensitivity[y], 0], [0, 0, 1/sensitivity[z]]])

    for row in range(3):
        averages = data_average(axes[row] + "+")
        #print(la.norm(np.dot(sensitivity_matrix, np.subtract(averages, bias))))
        for column in range(3):
            alignment[column, row] = la.norm(np.dot(sensitivity_matrix, (averages[column] - bias[column])))

    return alignment.transpose()

def test_calibration(bias, sensitivity, alignment):
    sensitivity_matrix = np.array([[1/sensitivity[x], 0, 0], [0, 1/sensitivity[y], 0], [0, 0, 1/sensitivity[z]]])

    dataset = read_data("az-") 
    not_moving = np.array([np.average(dataset[x]), np.average(dataset[y]), np.average(dataset[z])])
    moving = data_average("x+")

    # not_moving_value = alignment * sensitivity_matrix.transpose() * (np.array([[not_moving[x]], [not_moving[y]], [not_moving[z]]]) - np.array([[bias[x]], [bias[y]], [bias[z]]]))
    # moving_value = alignment * sensitivity_matrix.transpose() * (np.array([[moving[x]], [moving[y]], [moving[z]]]) - np.array([[bias[x]], [bias[y]], [bias[z]]]))
    not_moving_value = np.dot(alignment, np.dot(sensitivity_matrix, (np.array([[not_moving[x] - bias[x]], [not_moving[y] - bias[y]], [not_moving[z] - bias[z]]]))))
    moving_value = np.dot(alignment, np.dot(sensitivity_matrix, (np.array([[moving[x] - bias[x]], [moving[y] - bias[y]], [moving[z] - bias[z]]]))))
    test_value = np.dot(sensitivity_matrix, (np.array([[moving[x] - bias[x]], [moving[y] - bias[y]], [moving[z] - bias[z]]])))

    print(not_moving_value)
    print(moving_value)
    print(test_value)
    print(alignment)

def print_calibrated_output(raw_data, bias, sensitivity, alignment):
    # Create sensitivity matrix (diagonal matrix)
    sensitivity_matrix = np.diag(1/sensitivity)
    
    # Apply calibration equation: ω = CΩR * Sω⁻¹ * (uω - bω)
    # First subtract bias
    data_minus_bias = raw_data - bias
    
    # Then multiply by inverse sensitivity matrix
    scaled_data = np.dot(sensitivity_matrix, data_minus_bias)
    
    # Finally apply alignment matrix
    calibrated_output = np.dot(alignment, scaled_data)
    
    print("Calibrated angular velocity:")
    print(f"ωx: {calibrated_output[0]:.6f} deg/s")
    print(f"ωy: {calibrated_output[1]:.6f} deg/s")
    print(f"ωz: {calibrated_output[2]:.6f} deg/s")
    
    return calibrated_output

bias = bias_calibration()
print(bias)
sensitivity = sensitivity_calibration()
print(sensitivity)
alignment = alignment_calibration(bias, sensitivity)
#print(alignment)
#test_calibration(bias, sensitivity, alignment)
normed_alignment = np.array([[0.9995790,0.0019301,-0.0289514],[-0.0019301,0.9999982,0.0000279],[0.0289514,0.0000279,0.9995808]])
#print_calibrated_output(data_average("x+"), bias, sensitivity, np.identity(3))
#print_calibrated_output(data_average("x+"), bias, sensitivity, normed_alignment)

data = None
with open('../config.json', mode="r") as json_file:
    data = json.loads(json_file.read())
    data['ahrs']['gyro_calibrate']['bias'] = bias.tolist()
    data['ahrs']['gyro_calibrate']['sensitivity'] = sensitivity.tolist()

with open('../config.json', mode="w") as json_file:
    s = json.dump(data, json_file, indent=4)

