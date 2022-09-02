"""Better version than v1"""

import math
import numpy as np

# Classes

class Transformation:
    """ A class wrapper for a transformation matrix.
        Replaces classes Base and Point from previous version.
    """

    def __init__(self,matrix):
        self.matrix = matrix

    def rot(self):
        """Returns rotation submatrix
        """
        return self.matrix[0:3,0:3]

    def trans(self):
        """Returns translation vector
        """
        return self.matrix[0:3,3]

    def dot(self,other):
        """Returns dot product of this and the given Transformation
        """
        return Transformation(self.matrix.dot(other.matrix))

    def inv(self):
        """Returns inverse of this transformation
        """
        return Transformation(np.linalg.inv(self.matrix))

class Linear:
    """A representation of a linear function determined from one point and the slope of the line.
    
    !!! The line is in x|z plane and should be used for Points with y=0 !!!

    """

    def __init__(self, trans_vector, slope):
        """ trans_vector = translation vector representing x, y, and z coordinates f the point. y is ignored/ assumed to be 0!
            
            slope = slope of the line
        """
        self.x = trans_vector[0]
        self.z = trans_vector[2]
        self.k = slope
        self.intercept = -(self.k * self.x) + self.z

    def solve(self, x):
        """The actual Linear function, returns z coordinate on line with given x coordinate
        """

class Circle:
    """A class representation of a circle in the x|z plane.
    """
    
    def __init__(self, origin, radius):
        self.origin = origin
        self.radius = radius

    def intersect(self, other):
        """ Calculates intersecting points of this and the given circle. If Circles do not intersect, returns tuple (None, None)
        """

        if(self.origin[2] != other.origin[2]):
            k = - (self.origin[0] - other.origin[0])/(self.origin[2] - other.origin[2])
            l = ((self.radius**2 - other.radius**2) - (self.origin[0]**2 - other.origin[0]**2) - (self.origin[2]**2 - other.origin[2]**2)) / (-2 * (self.origin[2] - other.origin[2]))
            a = k**2 + 1
            b = 2 * k * (l - self.origin[2]) - 2 * self.origin[0]
            c = self.origin[0]**2 - self.radius**2 + (l - self.origin[2])**2

            x1 = (-b + math.sqrt(b**2 - 4 * a * c))/(2 * a)
            x2 = (-b - math.sqrt(b**2 - 4 * a * c))/(2 * a)
            if(x1 == x2):
                return np.array([x1, 0, k * x1 + l]),None # only one point of intersection
            else:
                return np.array([x1, 0, k * x1 + l]),np.array([x2, 0, k * x2 + l])
        elif(self.origin[0] != other.origin[0]):
            k = - (self.origin[2] - other.origin[2])/(self.origin[0] - other.origin[0])
            l = ((self.radius**2 - other.radius**2) - (self.origin[0]**2 - other.origin[0]**2) - (self.origin[2]**2 - other.origin[2]**2))/(-2 * (self.origin[0] - other.origin[0]))
            a = k**2 +1
            b = 2 * k * (l - self.origin[0]) - 2 * self.origin[2]
            c = self.origin[2]**2 - self.radius**2 + (l - self.origin[0])**2

            z1 = (-b + math.sqrt(b**2 - 4 * a * c))/(2 * a)
            z2 = (-b - math.sqrt(b**2 - 4 * a * c))/(2 * a)
            if(z1 == z2):
                return np.array([k * z1 + l, 0, z1]),None
            else:
                return np.array([k * z1 + l, 0, z1]),np.array([k * z2 + l, 0, z2])
        else: # no points or infinite points of intersection, circles have same origin
            return None, None

# Constants
# All lengths are in millimeters and all angles are in degrees

ROOT = Transformation(np.eye(4))
'''Center of robot'''

LEFT_LEG_ORIGIN = Transformation(np.array([[1, 0, 0, 0],
                                           [0, 1, 0, 102.5],
                                           [0, 0, 1, -14.5]]))
'''Center of left shoulder servo rotation axis in the root coordinate frame'''                                           

RIGHT_LEG_ORIGIN = Transformation(np.array([[1, 0, 0, 0],
                                            [0, 1, 0, -102.5],
                                            [0, 0, 1, -14.5]]))
'''Center of right shoulder servo rotation axis in the root coordinate frame'''   

T_A1_LEG = Transformation(np.array([[1, 0, 0, 60.2],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, -70.232],
                                    [0, 0, 0, 1]]))
'''Position of the center of the A1 servo rotation axis in relation to an [X]_LEG_ORIGIN'''   
# A1 acts as the origin in calculating the position of the end effector. 
# There is virtualy no difference between the left and right leg, so the same T_A1_LEG transformation can be used.
# After the position of the end-effector has been calculated, one need only use the needed LEG_ORIGIN T-matrix to obtain the position of the end-effector in the ROOT system

A2_IN_A1 = np.array([-100, 0, 25.232])
'''Position of the center of the A2 servo rotation axis in relation to A1'''

R_LEN = 80.642
'''Pivot lenght of servo A2'''
N1_LEN = 187.5
'''Length of the upper paralelogram'''
N2_LEN = 166.25
'''Length of the arm from A2a to C'''
F_LEN = 75.0
'''Length of foot'''
L_LEN = 60.0
'''Length of elbow at C'''
P_LEN = 187.5
'''Length of lower paralelogram'''
PHI = 150.0
'''Angle between elbow and lower leg'''

if __name__=="__main__":
    K1 = Circle(np.array([1,0,0]),2.0)
    K2 = Circle(np.array([-1,0,0]),2.0)
    P1, P2 = K1.intersect(K2)
    print(P1[0],",",P1[1],",",P1[2])
    print(P2[0],",",P2[1],",",P2[2])