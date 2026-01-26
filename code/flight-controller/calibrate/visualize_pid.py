import matplotlib.pyplot as plt
import numpy as np
import csv

file = 60

angle = np.array([])
pid = np.array([])
l= []

with open('../flight_logs/bench_tests/flight-log_' + str(file) + '.csv', 'r') as file:
    csv_file = csv.DictReader(file);

    counter = 0
    for line in csv_file:
        angle = np.append(angle, float(line['orientation_x']))
        pid = np.append(pid, float(line['pid_x']))
        l.append(float(line['pid_x']))
        counter = 0

    plt.plot(angle, color='orange')
    plt.plot(pid, color='green')
    plt.plot([0, len(angle)], [0, 0], color='blue')
    # plt.autoscale(True, axis='both')
    # plt.axis((0, len(angle), 0, 100))
    plt.show()

