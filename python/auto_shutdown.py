import time
from subprocess import call
from pymata4 import pymata4
board = pymata4.Pymata4()

total_t = 310 # change to value from timer.txt
wait_t = 300 # 5 min wait before auto shutdown

t = total_t - wait_t #seconds until auto_shutdown
time.sleep(t)

# buzz tune to inform user of empty battery and auto_shutdown

TONE_PIN = 9
notes = {"c":523,"d":587,"e":659,"f":698,"g":784,"a":880,"h":988,"c2":1047} #B E G A
tune = ["h","e","g","a"]

for note in tune:
    freq = notes[note]
    board.play_tone(TONE_PIN, freq, 250)
    time.sleep(0.25)

board.shutdown()

time.sleep(wait_t)

# shutdown
call("sudo poweroff", shell=True)
