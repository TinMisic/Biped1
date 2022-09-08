import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import iterpolation as inter
import math

def dist(a, b):
    return a - b

def adder(start, end, covered):
    return start + covered

int1 = inter.Interpolation(0, 1,dist,adder)
t = 10000
target = 10000
val = int1.go(target, t)
start = time.time()
first=False
second = False

def calc():
    global first
    global second
    val = int1.update()
    #print(val)
    if(val>=4000 and not first):
        int1.go(10000,t)
        first = True
    # if(val>=6000 and not second):
    #     int1.go(1000,t)
    #     second=True

    return val

def update(frame):
    x_data.append((time.time()-start))
    y_data.append(calc())
    line.set_data(x_data, y_data)
    figure.gca().relim()
    figure.gca().autoscale_view()
    return line,


x_data, y_data = [], []

figure = plt.figure()
line, = plt.plot_date(x_data, y_data, '-')
    

animation = FuncAnimation(figure, update, interval=10)

plt.show()