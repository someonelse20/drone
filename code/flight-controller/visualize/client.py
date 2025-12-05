import socket
import pickle

PORT = 5000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect(('drone', PORT))

class vector:
    x = 0.0
    y = 0.0
    z = 0.0

class sensors:
    gyro = vector()
    accel = vector()
    mag = vector()

def run(function):
    return_value = sensors

    data_recv = s.recv(1024).decode()
    data_recv = data_recv.split('\n')

    data_recv.pop()

    #print(data_recv)

    while len(data_recv) % 3 != 0:
        data_recv.pop()

    for data in data_recv:
        print(data)
        data_sensors = data.split(';')
        gyro_data = data_sensors[0].split(',')
        accel_data = data_sensors[1].split(',')
        mag_data = data_sensors[2].split(',')

        return_value.gyro.x = float(gyro_data[0])
        return_value.gyro.y = float(gyro_data[1])
        return_value.gyro.z = float(gyro_data[2])

        return_value.accel.x = float(accel_data[0])
        return_value.accel.y = float(accel_data[1])
        return_value.accel.z = float(accel_data[2])

        return_value.mag.x = float(mag_data[0])
        return_value.mag.y = float(mag_data[1])
        return_value.mag.z = float(mag_data[2])

    function(return_value)

