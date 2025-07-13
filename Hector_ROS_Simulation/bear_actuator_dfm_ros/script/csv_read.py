import csv
import matplotlib.pyplot as plt
import numpy as np

p0 = []
p1 = []
v0 = []
v1 = []
filename = '/home/kenji/step_motion.csv'
with open(filename, encoding='utf8', newline='') as f:
    csvread = csv.reader(f)
    for row in csvread:
        print(len(row), row)
        p0.append(float(row[0]))
        p1.append(float(row[1]))
        v0.append(float(row[2]))
        v1.append(float(row[3]))

    t = np.linspace(0, 0.6, int(0.6/0.02))

    fig = plt.figure(figsize=(10, 10))
    ax1 = fig.add_subplot(411)
    ax2 = fig.add_subplot(412)
    ax3 = fig.add_subplot(413)
    ax4 = fig.add_subplot(414)
    ax1.plot(t, p0)
    ax2.plot(t, p1)
    ax3.plot(t, v0)
    ax4.plot(t, v1)
    plt.xlabel('Time')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.legend()
    plt.show()
