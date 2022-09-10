"""Interpolates the movement of one foot between two points"""
import iterpolation as ip
import paralel_inv_kinematic_v2 as pik
import coord_class_3D_p_v2 as co
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def dist_op(a, b):
    return np.linalg.norm(a - b)

def adder_op(a, b, covered):
    unit = (b - a) / np.linalg.norm(b - a)
    return a + (covered * unit)

def equals(a,b):
    return (a==b).all()

points = [np.array([133.5678, 0, -398.957424, 1]), np.array([-21.74925, 0, -399.9489, 1]), np.array([-2.37624, 0, -341.70617, 1])]

for i in range(len(points)):
    points[i] = (co.RIGHT_LEG_ORIGIN.matrix.dot(points[i]))[0:3]

t = 1000 # ms
inc = 1 # mm

ip1 = ip.Interpolation(points[0],10,dist_op,adder_op,equals)

index = 0

al1_data, al2_data, al0_data, t_data = [], [], [], []
start=time.time()
loop = False
while(True):
    if index==1 and loop == True:break
    if(ip1.isFinished()):
        index = (index+1) % 3
        if index == 0: loop = True
        ip1.go(points[index],t)

    val = ip1.update()
    al1, al2, al3 = pik.getAlphas(val,'r')
    print(al1," ",al2," ",al3)

    al0_data.append(al1)
    al1_data.append(al2)
    al2_data.append(al3)
    t_data.append(time.time()-start)
    time.sleep(0.05)

# def update(frame):
#     global index, al1_data, al2_data, al0_data, t_data
#     if(ip1.isFinished()):
#         index = (index+1)%3
#         ip1.go(points[index],t)
#         if index==0: # reset plot
#             al1_data, al2_data, al0_data, t_data = [], [], [], []

#     val = ip1.update()
#     al1, al2, al3, _ = pik.getAlphas(val,'r')

#     al0_data.append(al1)
#     al1_data.append(al2)
#     al2_data.append(al3)
#     t_data.append(time.now())
#     line1.set_data(al0_data, t_data)
#     line2.set_data(al1_data, t_data)
#     line3.set_data(al2_data, t_data)
#     figure.gca().relim()
#     figure.gca().autoscale_view()
#     return line1,line2,line3,


figure = plt.figure()
line1, = plt.plot(t_data, al0_data, '-',color='r')
line2, = plt.plot(t_data, al1_data, '-',color='g')
line3, = plt.plot(t_data, al2_data, '-',color='b')

# plt.xlim([-50, 150])
# plt.ylim([-425, -300])
plt.show()