import time

print("Starting timer...")
secs = 0
inc = 60 # increment sec by inc for each loop

while(True):
    try:
        save = open("timer.txt", 'w')
    except FileNotFoundError:
        save = open("timer.txt", 'x')

    hours = secs // 3600
    minutes = secs % 3600 // 60
    # seconds = secs % 3600 % 60
    line = str(hours) + "h " + str(minutes) + "m " #+ str(seconds) + "s"

    save.write(line)
    save.close()

    time.sleep(inc)
    secs += inc