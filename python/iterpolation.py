import time

class Interpolation:
    """Interpolates between initial value and final value in given duration."""

    def __init__(self, initVal, increment, dist_op, adder_op):
        """Initializer of initial value and value increment(float)
        
           dist_op(val1, val2) is the given function with which to calculate distance between values
           adder_op(startVal, endVal, covered) is the given function which adds the covered value from startValue to endValue. Returns current value
        """
        self.startVal = initVal
        self.currVal = initVal
        self.endVal = initVal
        self.increment = increment
        
        self.dist_op = dist_op
        self.adder_op = adder_op

        self.speed = 0
        self.start = time.time() * 1000

    def go(self, newVal, duration):
        """Starts new interpolation to new value starting at current value."""
        self.duration = duration
        self.endVal = newVal
        self.startVal = self.currVal
        self.start = time.time() * 1000

        self.accel = - 2 * self.dist_op(self.endVal, self.currVal) / duration**2
        self.speed = - self.accel * duration
        return self.currVal


    def update(self):
        """Updates current value and returns it as output"""
        currTime = time.time() * 1000
        elapsed  = currTime - self.start
        if(self.currVal != self.endVal):
            covered = (self.speed * elapsed + self.accel * elapsed**2 / 2) // self.increment * self.increment
            self.currVal = self.adder_op(self.startVal, self.endVal, covered)
            if(elapsed >= self.duration):
                self.currVal = self.endVal
                self.speed = 0
                self.accel = 0

        return self.currVal
