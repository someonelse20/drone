import numpy as np
from numpy import linalg as la

x = 0 
y = 1 
z = 2

axes = ["x", "y", "z"]

MAGNETIC_FIELD = 0.520411

def read_data(file): # Converts raw imu output into a three dementional numpy array of the gyrometer data.
    data = np.array([])

    datafile = open("./data/" + file, "rt")
    rawdata = datafile.read().split("\n")

    for line in rawdata:
        if "mag" in line:
            nums = line.replace("mag:", "").split(",")
            dataline = []
            for num in nums:
                dataline.append(float(num))

            if data.size == 0:
                data = np.array([dataline])
            else:
                data = np.append(data, [dataline], axis=0)

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

    #print(np.allclose(np.dot(np.dot(M, la.pinv(M)),v), v))

    #print(h)

    #print(v)
    #print(np.dot(M, h))

    return np.array([h[0], h[2], h[4]])

def ellipsoid_calibration(files=["m1", "m2", "m3", "m4", "m5"]):
    """
    dataset = read_data("m1")
    dataset = np.append(dataset, read_data("m2"), axis=0)
    dataset = np.append(dataset, read_data("m3"), axis=0)
    """

    dataset = np.array([])
    for file in files:
        if dataset.size == 0:
            dataset = read_data("m1")
        else:
            dataset = np.append(dataset, read_data(file), axis=0)

    x = dataset[:, 0]
    y = dataset[:, 1]
    z = dataset[:, 2]

    D = np.column_stack([
        x**2 + y**2 - 2*z**2,
        x**2 + z**2 - 2*y**2,
        2*x*y,
        2*x*z,
        2*y*z,
        2*x,
        2*y,
        2*z,
        np.ones_like(x)
    ])

    d2 = x**2 + y**2 + z**2
    u = la.lstsq(D, d2, rcond=None)[0]

    #print(D)
    #print(d2)

    v = np.zeros(10)
    v[0] = u[0] + u[1] - 1
    v[1] = u[0] - 2*u[1] - 1
    v[2] = u[1] - 2*u[0] - 1
    v[3:10] = u[2:9]

    v = v.reshape(-1, 1)

    A = np.array([
        [v[0][0], v[3][0], v[4][0], v[6][0]],
        [v[3][0], v[1][0], v[5][0], v[7][0]],
        [v[4][0], v[5][0], v[2][0], v[8][0]],
        [v[6][0], v[7][0], v[8][0], v[9][0]]
    ])

    center = -np.linalg.solve(A[:3, :3], v[6:9]).reshape(-1, 1)

    T = np.eye(4)
    T[3, :3] = center.flatten()

    R = T @ A @ T.T

    evals, evecs = la.eig(R[:3, :3] / (-R[3, 3]))

    radii = np.sqrt(1 / np.abs(evals))
    sgns = np.sign(evals)
    radii = radii * sgns

    soft_iorn = evecs.T @ np.diag([MAGNETIC_FIELD / radii[0], MAGNETIC_FIELD / radii[0], MAGNETIC_FIELD / radii[0]]) @ evecs
    hard_iorn = soft_iorn @ center

    return soft_iorn, np.array([hard_iorn[0][0], hard_iorn[1][0], hard_iorn[2][0]])

def alignment_calibration(files):
    init_soft_iorn, init_hard_iorn = ellipsoid_calibration()

    v = np.array([])

    for axis in range(3):
        dataset = np.array([])
        if dataset.size == 0:
            dataset = calibrate(read_data(files[axis]), init_soft_iorn, init_hard_iorn)
        else:
            dataset = np.append(dataset, calibrate(read_data(files[axis]), init_soft_iorn, init_hard_iorn), axis=0)

        x = dataset[:, 0]
        y = dataset[:, 1]
        z = dataset[:, 2]

        M = np.column_stack([
            x,
            y,
            z
        ])

        ones = np.ones((x.size, 1))

        if v.size == 0:
            v = la.lstsq(M, ones)[0]
        else:
            v = np.append(v, la.lstsq(M, ones)[0], axis=1)

    R = np.array([v[0] / la.norm(v[0]), v[1] / la.norm(v[1]), v[2] / la.norm(v[2])])

    S = R.transpose() @ init_soft_iorn

    H = R.transpose() @ init_hard_iorn

    print(S)
    print(H)

    return S, H

def calibrate(points, soft_iorn, hard_iorn):
    return_list = np.array([])

    for i in range(len(points)):
        if return_list.size == 0:
            return_list = np.array([np.dot(soft_iorn, points[i]) - hard_iorn])
        else:
            return_list = np.append(return_list, [np.dot(soft_iorn, points[i]) - hard_iorn], axis=0)

    return return_list



offset = offset_calibration()
#print(offset)

callibrate = alignment_calibration(["gx+", "gy+", "gz+"])
#print(callibrate)

