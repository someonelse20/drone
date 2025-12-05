import matplotlib.pyplot as plt
import client

x = []
y = []

count = 1

plt.ion()

graph = plt.plot([0, 0], [0, 1])[0]

def update(data):
    global x, y, count, graph

    x.append(count)
    y.append(data.gyro.x)

    graph.remove()

    graph = plt.plot(x, y)[0]
    if count >= 50: plt.xlim(count - 50, count) 
    else: plt.xlim(0, count)
    #plt.ylim(-10, 10)

    count += 1

    plt.pause(0.05)

while 1:
    client.run(update)

client.s.close()

