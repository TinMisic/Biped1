"""Rudimentary skeleton of the main program"""
print("Main program started...")

import numpy as np
import time
import iterpolation as ip
import paralel_inv_kinematic_v2 as pik
import paralel_kinematic_v2 as pk
import coord_class_3D_p_v2 as co
from pymata4 import pymata4
import sys

# function definitions
def angle_correct(alpha, leg):
    if(leg=="l"):
        return int((-alpha - offset + center) * scale)
    elif(leg=="r"):
        return int((alpha + offset + center) * scale)
    else:
        return int((alpha + center) * scale)

def move_servos(left_leg_point,right_leg_point):
    #left leg ik
    l0, l1, l2 = pik.getAlphas(left_leg_point,"l")
    l0 = angle_correct(l0,"c")
    l1 = angle_correct(l1,"l")
    l2 = angle_correct(l2,"l")

    #right leg
    r0, r1, r2 = pik.getAlphas(right_leg_point,"r")
    r0 = angle_correct(r0,"c")
    r1 = angle_correct(r1,"r")
    r2 = angle_correct(r2,"r")

    board.servo_write(pins["L0"], l0)
    board.servo_write(pins["L1"], l1)
    board.servo_write(pins["L2"], l2)
    board.servo_write(pins["R0"], r0)
    board.servo_write(pins["R1"], r1)
    board.servo_write(pins["R2"], r2)

def dist_op(a, b):
    return np.linalg.norm(a - b)

def adder_op(a, b, covered):
    unit = (b - a) / np.linalg.norm(b - a)
    return a + (covered * unit)

def equals(a,b):
    return (a==b).all()

# initialization
board = pymata4.Pymata4()
center = 135
offset = 90
scale = 180/270
pins = {"L0":2, "L1":3, "L2":4, "R0":5, "R1":6, "R2":7}

# servo initialization and home pose
for servo in pins.keys():
    board.set_pin_mode_servo(pins[servo])

# servo home pose
home = [pk.getD(0, -135, -180, "l"),pk.getD(0, -135, -180, "r")]
move_servos(home[0], home[1])

# other constants
pointsL = [(co.LEFT_LEG_ORIGIN.matrix.dot(x))[0:3] for x in [np.array([133.5678, 0, -398.957424, 1]), np.array([-21.74925, 0, -399.9489, 1]), np.array([-2.37624, 0, -341.70617, 1])]]
pointsR = [(co.RIGHT_LEG_ORIGIN.matrix.dot(x))[0:3] for x in [np.array([133.5678, 0, -398.957424, 1]), np.array([-21.74925, 0, -399.9489, 1]), np.array([-2.37624, 0, -341.70617, 1])]]

t = 1000 # ms
sleep = 0.025 # s
inc = 1 # mm

ipL = ip.Interpolation(home[0],10,dist_op,adder_op,equals)
ipR = ip.Interpolation(home[1],10,dist_op,adder_op,equals)

indexL = 0
indexR = 1

print("Initialization complete.")
# main loop

try:
    # move legs from starting position to initial points in gait
    ipL.go(pointsL[indexL],t)
    ipR.go(pointsR[indexR],t)

    while(True):
        # update left leg
        if(ipL.isFinished()):
            indexL = (indexL + 1) % 3
            ipL.go(pointsL[indexL],t)
        
        # update right leg
        if(ipR.isFinished()):
            indexR = (indexR + 1) % 3
            ipR.go(pointsR[indexR],t)

        # get positions
        l = ipL.update()
        r = ipR.update()

        move_servos(l,r)

        time.sleep(sleep)
        
except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)