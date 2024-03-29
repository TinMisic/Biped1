import math
import numpy as np
#from mpl_toolkits.mplot3d import axes3d
#import matplotlib.pyplot as plt
import plotly.express as px

#Classes

class Base:
    """A class that contains three vectors of a base.
    
        Methods:
        __init__(self,x,y,z)
        rotateAxis(self,angle,axis)

    """
    def __init__(self,x,y,z):
        self.x=x
        self.y=y
        self.z=z

    def rotateAxis(self,angle,axis):
        """Rotates around given axis
        
        AXIS MUST BE UNIT VECTOR"""
        radAng=math.radians(angle)

        #rotation matrix
        A=np.array(([math.cos(radAng) + math.pow(axis[0],2) * (1-math.cos(radAng)) , axis[0]*axis[1]*(1-math.cos(radAng))-axis[2]*math.sin(radAng), axis[0]*axis[2]*(1-math.cos(radAng)) + axis[1]*math.sin(radAng)],
        [axis[1]*axis[0]*(1 - math.cos(radAng)) + axis[2]*math.sin(radAng), math.cos(radAng) + math.pow(axis[1],2)*(1-math.cos(radAng)),axis[1]*axis[2]*(1 - math.cos(radAng)) - axis[0]*math.sin(radAng)],
        [axis[2]*axis[0]*(1 - math.cos(radAng)) - axis[1]*math.sin(radAng),axis[2]*axis[1]*(1 - math.cos(radAng)) + axis[0]*math.sin(radAng),math.cos(radAng)+math.pow(axis[2],2)*(1-math.cos(radAng))]))

        xResult=A.dot(self.x)
        yResult=A.dot(self.y)
        zResult=A.dot(self.z)

        return Base(xResult,yResult,zResult)

class Point:
    """A representation of a point with coordinates x, y and z 
        ,angle alpha which was used to get said coordinates, and angle phi that represents rotation around the z axis

        Methods:

        -init

        -dist: Returns thistance from this point to another

        -translate: translates this Point to given Point

        -rotateX:rotates around x axis

        -rotateY:rotates around y axis

        -rotateZ:rotates around z axis
    """
    def __init__(self,x,y,z,alpha=0.0,phi=0.0,ownBase=Base(np.array([1,0,0]),np.array([0,1,0]),np.array([0,0,1]))):
        self.x=x
        self.y=y
        self.z=z
        self.alpha=alpha
        self.phi=phi
        self.ownBase=ownBase
    
    def dist(self,B):
        return math.sqrt((self.x-B.x)**2 + (self.y-B.y)**2 + (self.z-B.z)**2)

    def translate(self,toPoint):
        self.x+=toPoint.x
        self.y+=toPoint.y
        self.z+=toPoint.z
        
        return self

    def rotateZ(self,angle):
        '''Rotates around z axis'''
        phi=math.radians(angle)
        operator=np.array([[math.cos(phi),-math.sin(phi),0],[math.sin(phi),math.cos(phi),0],[0,0,1]])
        point_arr=np.array([self.x,self.y,self.z])

        rotated=np.matmul(operator,point_arr)
        result=Point(rotated[0],rotated[1],rotated[2],self.alpha,angle,self.ownBase)

        return result

    def rotateX(self,angle):
        '''Rotates around x axis'''
        phi=math.radians(angle)
        operator=np.array([[1,0,0],[0,math.cos(phi),-math.sin(phi)],[0,math.sin(phi),math.cos(phi)]])
        point_arr=np.array([self.x,self.y,self.z])

        rotated=np.matmul(operator,point_arr)
        self.x=rotated[0]
        self.y=rotated[1]
        self.z=rotated[2]

        return self

    def rotateY(self,angle):
        '''Rotates around y axis'''
        phi=math.radians(angle)
        operator=np.array([[math.cos(phi),0,-math.sin(phi)],[0,1,0],[math.sin(phi),0,math.cos(phi)]])
        point_arr=np.array([self.x,self.y,self.z])

        rotated=np.matmul(operator,point_arr)
        self.x=rotated[0]
        self.y=rotated[1]
        self.z=rotated[2]

        return self

    def rotateAxis(self,angle,axis):
        """Rotates around given axis
        
        AXIS MUST BE UNIT VECTOR"""
        radAng=math.radians(angle)

        #rotation matrix
        A=np.array(([math.cos(radAng) + math.pow(axis[0],2) * (1-math.cos(radAng)) , axis[0]*axis[1]*(1-math.cos(radAng))-axis[2]*math.sin(radAng), axis[0]*axis[2]*(1-math.cos(radAng)) + axis[1]*math.sin(radAng)],
        [axis[1]*axis[0]*(1 - math.cos(radAng)) + axis[2]*math.sin(radAng), math.cos(radAng) + math.pow(axis[1],2)*(1-math.cos(radAng)),axis[1]*axis[2]*(1 - math.cos(radAng)) - axis[0]*math.sin(radAng)],
        [axis[2]*axis[0]*(1 - math.cos(radAng)) - axis[1]*math.sin(radAng),axis[2]*axis[1]*(1 - math.cos(radAng)) + axis[0]*math.sin(radAng),math.cos(radAng)+math.pow(axis[2],2)*(1-math.cos(radAng))]))

        selfVector=np.array(([self.x,self.y,self.z]))

        result=A.dot(selfVector)

        return result

    def reduce(self):
        """If any of the point's values are less than 1e-6, 
        reduce them to 0
        """
        if math.fabs(self.x)<=1e-6:
            self.x=0
        if math.fabs(self.y)<=1e-6:
            self.y=0
        if math.fabs(self.z)<=1e-6:
            self.z=0
        if math.fabs(self.alpha)<=1e-6:
            self.alpha=0
        if math.fabs(self.phi)<=1e-6:
            self.phi=0
    
        
class Linear:
    """A representation of a linear function determined from one Point and the sope of the line.
    
    !!!The line is in x|z plane and should be used for Points with y=0!!!
    
        Methods:

        -init

        -get_Y: solves the linear function for given x
    """
    def __init__(self,X,slope):
        self.X=Point(X.x,X.y,X.z,X.alpha,X.phi,X.ownBase)
        self.k=slope
        self.intercept=-(self.k * X.x) + X.z

    def get_Z(self,xCoord):#the actual Linear function, returns Point on line with given x coordinate
        zCoord=self.k*xCoord + self.intercept

        return zCoord

class Circle:
    """A class representation of a circle"""
    def __init__(self,O,r):
        self.origin=O
        self.radius=r
    
    def intersect(self,other):
        if(self.origin.z != other.origin.z):
            k = - (self.origin.x - other.origin.x)/(self.origin.z - other.origin.z)
            l = ((self.radius**2 - other.radius**2) - (self.origin.x**2 - other.origin.x**2) - (self.origin.z**2 - other.origin.z**2))/(-2 * (self.origin.z - other.origin.z))
            a = k**2 +1
            b = 2 * k * (l - self.origin.z) - 2 * self.origin.x
            c = self.origin.x**2 - self.radius**2 + (l - self.origin.z)**2

            x1 = (-b + math.sqrt(b**2 - 4 * a * c))/(2 * a)
            x2 = (-b - math.sqrt(b**2 - 4 * a * c))/(2 * a)
            if(x1 == x2):
                return Point(x1, 0, k * x1 + l),None #only one point of intersection
            else:
                return Point(x1, 0, k * x1 + l),Point(x2, 0, k * x2 + l)
        elif(self.origin.x != other.origin.x):
            k = - (self.origin.z - other.origin.z)/(self.origin.x - other.origin.x)
            l = ((self.radius**2 - other.radius**2) - (self.origin.x**2 - other.origin.x**2) - (self.origin.z**2 - other.origin.z**2))/(-2 * (self.origin.x - other.origin.x))
            a = k**2 +1
            b = 2 * k * (l - self.origin.x) - 2 * self.origin.z
            c = self.origin.z**2 - self.radius**2 + (l - self.origin.x)**2

            z1 = (-b + math.sqrt(b**2 - 4 * a * c))/(2 * a)
            z2 = (-b - math.sqrt(b**2 - 4 * a * c))/(2 * a)
            if(z1 == z2):
                return Point(k * z1 + l, 0, z1 ),None
            else:
                return Point(k * z1 + l, 0, z1),Point(k * z2 + l, 0, z2)
        else:
            return None, None #no points or infinite points of intersection, circles have same origin

class Pose:
    """A structure that contains three Points that make up one movement or Pose"""
    def __init__(self,A,B,C):
        self.A=A
        self.B=B
        self.C=C

    def printp(self):
        print("A:"+(str)(self.A.x)+", "+(str)(self.A.y)+", "+(str)(self.A.z)+"   alpha1:"+(str)(self.A.alpha)+"   phi1:"+(str)(self.A.phi))
        print("B:"+(str)(self.B.x)+", "+(str)(self.B.y)+", "+(str)(self.B.z)+"   alpha2:"+(str)(self.B.alpha)+"   phi2:"+(str)(self.B.phi))
        print("C:"+(str)(self.C.x)+", "+(str)(self.C.y)+", "+(str)(self.C.z)+"   alpha3:"+(str)(self.C.alpha)+"   phi3:"+(str)(self.C.phi))

    def plot(self):
        X=[root.x,self.A.x,self.B.x,self.C.x]
        Y=[root.y,self.A.y,self.B.y,self.C.y]
        Z=[root.z,self.A.z,self.B.z,self.C.z]

        fig = px.line_3d(x=X,y=Y,z=Z)
        fig.update_layout(scene=dict(
                 aspectmode='data'))
        fig.show()
        

#Constants:
i=np.array([1,0,0])#root base vector 
j=np.array([0,1,0])#root base vector
k=np.array([0,0,1])#root base vector
norm=Base(i,j,k)
root=Point(0,0,0,0,0,norm)# root point
r0x = -2 #distance of A2 from A1 on x axis
r0z = 0.50464 #distance of A2 from A1 on z axis
r = 1.61286 #pivot length of servo 2
n1 = 5 #length of upper paralelogram
n2 = 4.75 #length of arm from A2a to C
f = 1.5 #length of foot
l = 1.2 #length of elbow at C
p = 5 #distance from C to D
phi = 150 #angle between elbow and lower leg


if __name__=="__main__":
    A=Point(60.2,0,-70.232)
    B=Point(40.56984,0,-410.2069304)
    K1 = Circle(A,187.5)
    K2 = Circle(B,187.5)
    P1,P2=K1.intersect(K2)
    print(P1.x,",",P1.y,",",P1.z)
    print(P2.x,",",P2.y,",",P2.z)

    