import sys
import time

print("Initializing pymata4...\n")
from pymata4 import pymata4

"""
This test is used to determine the strength of one leg (left leg / [L] ).
Servo L0 is not tested.
"""

board = pymata4.Pymata4()

try:
    print('setting servos...')
    board.set_pin_mode_servo(2) #L1
    board.set_pin_mode_servo(3) #L2
    
    scale=180/270
    L1_off = 8.5
    L2_off = 0.5
    L1_val = 120
    L2_val = 180

    while(True):
        L1_val = int(input("set L1>"))
        L2_val = int(input("set L2>"))
        board.servo_write(2, int(L1_val * scale + L1_off))
        board.servo_write(3, int(L2_val * scale + L2_off))
        print("L1:" + str(L1_val) + " " + "L2:" + str(L2_val) + "\n")

except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)
