import pathlib 
import ctypes
import socket
import time

PORT = 5000

class vector:
    x = 0.0
    y = 0.0
    z = 0.0

class sensors:
    gyro = vector()
    accel = vector()
    mag = vector()

class cvector (ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]

libname = pathlib.Path().absolute() / "build/libget_data.so"
get_data = ctypes.CDLL(libname)
get_data.read_gyro.restype = cvector
get_data.read_accel.restype = cvector
get_data.read_mag.restype = cvector

get_data.imu_init(ctypes.c_int(8), ctypes.c_int(500), ctypes.c_int(0))

gyro = get_data.read_gyro()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', PORT))
print('socket bound to port ' + str(PORT))
s.listen(5)
print('socket is listening')

c, addr = s.accept()
print('connected to ', addr)

while True:
    # c.send(b'hello world!')

    gyro = get_data.read_gyro()
    accel = get_data.read_accel()
    mag = get_data.read_mag()

    # data = {'gyro': [gyro.x, gyro.y, gyro.z], 'accel': [accel.x, accel.y, accel.z], 'mag': [mag.x, mag.y, mag.z]}

    data = str(gyro.x) + ',' + str(gyro.y) + ',' + str(gyro.z) + ';'
    data += str(accel.x) + ',' + str(accel.y) + ',' + str(accel.z) + ';'
    data += str(mag.x) + ',' + str(mag.y) + ',' + str(mag.z) + '\n'

    """
    data = sensors()

    data.gyro.x = gyro.x
    data.gyro.y = gyro.y
    data.gyro.z = gyro.z

    data.accel.x = accel.x
    data.accel.y = accel.y
    data.accel.z = accel.z

    data.mag.x = mag.x
    data.mag.y = mag.y
    data.mag.z = mag.z
    """

    c.sendall(data.encode())

    #time.sleep(0.1)

c.close()

