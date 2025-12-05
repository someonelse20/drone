import pathlib 
import ctypes
import socket

PORT = 5000

class vector (ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]

libname = pathlib.Path().absolute() / "build/libget_data.so"
get_data = ctypes.CDLL(libname)
get_data.read_gyro.restype = vector
get_data.read_accel.restype = vector
get_data.read_mag.restype = vector

get_data.imu_init(ctypes.c_int(8), ctypes.c_int(500), ctypes.c_int(0))

gyro = get_data.read_gyro()

s = socket.socket()
s.bind(('', PORT))
print('socket bound to port ' + str(PORT))
s.listen(5)
print('socket is listening')

while True:
    c, addr = s.accept()
    print('connected to ' + addr)

    while True:
        c.send(b'hello world!')
        break

    c.close()

