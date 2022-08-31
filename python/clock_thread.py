#This script will serve as the skeleton for main. One thread will be the main kinematics and control thread, while one will be the clock. The clock initiates a shutdown after a certain amount of time.

import threading
import time
from subprocess import call

# from pymata4 import pymata4
# board = pymata4.Pymata4()
# TONE_PIN = 9
# notes = {"c":523,"d":587,"e":659,"f":698,"g":784,"a":880,"h":988,"c2":1047} #B E G A
# tune = ["b","e","g","a"]

exitFlag = 0

class Countdown (threading.Thread):
    def __init__(self, total_time, wait_time):
        threading.Thread.__init__(self)
        self.total_time = total_time
        self.wait_time = wait_time

    def run(self):
        time.sleep(self.total_time - self.wait_time)

        #play tune
        print(str(self.wait_time) + " seconds until shutdown...")
        # for note in tune:
        #     freq = notes[note]
        #     board.play_tone(TONE_PIN, freq, 500)
        #     time.sleep(0.5)
        time.sleep(self.wait_time)
        
        board_shutdown()
        print("poweroff")
        #call("sudo poweroff", shell = True)

def board_shutdown():
    print("Shutting down Arduino board...")
    #board.shutdown()

# Create new threads
cntdwn = Countdown(30,10)

# Start new threads
cntdwn.setDaemon(True)
cntdwn.start()

# kill countdown upon recieving signal

# call board_shutdown when SIGTERM or SIGKILL recieved

# main run below
print("Exiting main.")