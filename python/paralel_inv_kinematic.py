import math
import numpy as np
import coord_class_3D_p as co

A1 = co.Point(0,0,0)
A2 = co.Point(A1.x + co.r0x, 0, A1.z + co.r0z)

def isAcceptable(A, B, C, direct):
    '''direct: left is -1, right is 1'''
    vec1 = co.Point(A.x - B.x, 0, A.z - B.z)
    vec2 = co.Point(C.x - B.x, 0, C.z - B.z)

    angle = 90
    if(vec1.x != 0):
        angle = math.degrees(math.atan(vec1.z / vec1.x))
        if(angle < 0):
            angle += 180
    
    vec2.rotateY(90 - angle)
    if((direct == 1 and vec2.x <= 0) or (direct == -1 and vec2.x >= 0)):
        return True
    return False

def getAlphas(D):
    alpha1 = 0
    alpha2 = 0
    points = dict()
    if(A1.dist(D) > co.n1 + co.p):
        raise ValueError("D is unreachable.")
    
    K1 = co.Circle(A1, co.n1)
    K2 = co.Circle(D, co.p)
    B11,B12 = K1.intersect(K2)
    if(B11==None and B12==None):
        raise ValueError("D must not be equal to A1.")
    elif(B11!=None and B12==None):
        B1 = co.Point(B11.x,B11.y,B11.z)
        if(A1.x != B1.x):
            sl_n1 = (A1.z - B1.z) / (A1.x - B1.x)
            alpha1 = math.degrees(math.atan(sl_n1)) 
        else:
            alpha1 = -90

        if(D.x != B1.x):
            sl_p = (D.z - B1.z) / (D.x - B1.x)
            beta1 = math.degrees(math.atan(sl_p))
        else:
            beta1 = -90
        
        xc = B1.x - co.f + co.l * math.cos(math.radians(beta1 - co.phi))
        zc = B1.z + co.l * math.sin(math.radians(beta1 - co.phi))
        C = co.Point(xc, 0, zc)

        K3 = co.Circle(A2, co.r)
        K4 = co.Circle(C, co.n2)

        A2a1,A2a2 = K3.intersect(K4)

        if(A2a1==None and A2a2==None):
            raise ValueError("Error in n2")
        elif(A2a1!=None and A2a2==None):
            A2a = co.Point(A2a1.x, A2a1.y, A2a1.z)

            if(A2a.x != A2.x):
                sl_r = (A2.z - A2a.z) / (A2.x - A2a.x)
                alpha2 = math.degrees(math.atan(sl_r)) 
            else:
                alpha2 = -90

            points["B1"] = B1
            points["C"] = C
            points["A2a"] = A2a
        else:
            if(isAcceptable(A2, A2a1, C,-1)):
                A2a = co.Point(A2a1.x,A2a1.y,A2a1.z)
            else:
                A2a = co.Point(A2a2.x,A2a2.y,A2a2.z)

            if(A2a.x != A2.x):
                sl_r = (A2.z - A2a.z) / (A2.x - A2a.x)
                alpha2 = math.degrees(math.atan(sl_r)) 
            else:
                alpha2 = -90

            points["B1"] = B1
            points["C"] = C
            points["A2a"] = A2a
    else:
        if(isAcceptable(A1,B11,D,-1)):
            B1 = co.Point(B11.x,B11.y,B11.z)
        else:
            B1 = co.Point(B12.x,B12.y,B12.z)

        if(A1.x != B1.x):
            sl_n1 = (A1.z - B1.z) / (A1.x - B1.x)
            alpha1 = math.degrees(math.atan(sl_n1)) 
        else:
            alpha1 = -90

        if(D.x != B1.x):
            sl_p = (D.z - B1.z) / (D.x - B1.x)
            beta1 = math.degrees(math.atan(sl_p))
        else:
            beta1 = -90
        
        xc = B1.x - co.f + co.l * math.cos(math.radians(beta1 - co.phi))
        zc = B1.z + co.l * math.sin(math.radians(beta1 - co.phi))
        C = co.Point(xc, 0, zc)

        K3 = co.Circle(A2, co.r)
        K4 = co.Circle(C, co.n2)

        A2a1,A2a2 = K3.intersect(K4)

        if(A2a1==None and A2a2==None):
            raise ValueError("Error in n2")
        elif(A2a1!=None and A2a2==None):
            A2a = co.Point(A2a1.x, A2a1.y, A2a1.z)

            if(A2a.x != A2.x):
                sl_r = (A2.z - A2a.z) / (A2.x - A2a.x)
                alpha2 = math.degrees(math.atan(sl_r)) 
            else:
                alpha2 = -90

            points["B1"] = B1
            points["C"] = C
            points["A2a"] = A2a
        else:
            if(isAcceptable(A2, A2a1, C,-1)):
                A2a = co.Point(A2a1.x,A2a1.y,A2a1.z)
            else:
                A2a = co.Point(A2a2.x,A2a2.y,A2a2.z)

            if(A2a.x != A2.x):
                sl_r = (A2.z - A2a.z) / (A2.x - A2a.x)
                alpha2 = math.degrees(math.atan(sl_r)) 
            else:
                alpha2 = -90

            points["B1"] = B1
            points["C"] = C
            points["A2a"] = A2a

    return alpha1,alpha2,points

if __name__=="__main__":
    D = co.Point(3.8971, 0, -12.2822420162)
    a1,a2,points=getAlphas(D)
    print("a1:",a1,", a2:",a2)
    for key in points.keys():
        print(key,":",points[key].x,",",points[key].z)