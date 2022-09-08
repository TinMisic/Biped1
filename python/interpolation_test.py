import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import iterpolation as inter
import math

def dist(a, b):
    return a - b

def adder(start, end, covered):
    return start + covered

def eq_op(a, b):
    return a == b

int1 = inter.Interpolation(0, 0.01,dist,adder,eq_op)
t = 12000
target = 100
val = int1.go(target, t/4)
start = time.time()
cleared = [False,False,False,False]

def calc(ts):
    global cleared
    val = int1.update()
    # print(val)
    if ts>16 and not cleared[3]:
        int1.go(-500,t)
        cleared[3] = True
    elif ts>10 and not cleared[2]:
        int1.go(0,t)
        cleared[2] = True
    elif ts>6 and not cleared[1]:
        int1.go(200,t/2)
        cleared[1] = True
    elif ts>4 and not cleared[0]:
        int1.go(-300,t/4)
        cleared[0] = True

    return val

def update(frame):
    x_data.append((time.time()-start))
    y_data.append(calc(time.time()-start))
    line.set_data(x_data, y_data)
    figure.gca().relim()
    figure.gca().autoscale_view()
    return line,


x_data, y_data = [], []

figure = plt.figure()
line, = plt.plot_date(x_data, y_data, '-')
    

animation = FuncAnimation(figure, update, interval=10)

plt.show()