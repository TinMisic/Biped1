import math
import coord_class_3D as co

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
    if(A2.dist(D) > co.r2 + (co.n2 + co.p1)):
        raise ValueError("D is unreachable.")
    K1 = co.Circle(A2, co.r2)
    K2 = co.Circle(D, co.n2 + co.p1)
    B21, B22 = K1.intersect(K2)
    if(B21==None and B22==None):
        raise ValueError("D must not be equal to A1 or A2.")
    elif(B21!=None and B22==None):
        B2 = co.Point(B21.x,B21.y,B21.z)
        xc = (co.p1 * B2.x + co.n2 * D.x)/(co.n2 + co.p1)
        zc = (co.p1 * B2.z + co.n2 * D.z)/(co.n2 + co.p1)
        C = co.Point(xc, 0, zc)

        K3 = co.Circle(C, co.n1)
        K4 = co.Circle(A1, co.r1)
        B11, B12 = K3.intersect(K4)
        if(B11==None and B12==None):
            raise ValueError("C==A1")
        elif(B11!=None and B12==None):
            B1 = co.Point(B11.x,B11.y,B11.z)
            #ready to calculate alpha 1 and alpha 2
            if(A1.x != B1.x):
                r1sl =  (A1.z - B1.z) / (A1.x - B1.x)#slope of r1
                alpha1 = math.degrees(math.atan(r1sl))
                if(not isAcceptable(A1,B1,C,1)):
                    raise ValueError("Error in knee bend.")
            else:
                alpha1 = -90
                if(not isAcceptable(A1,B1,C,1)):
                    raise ValueError("Error in knee bend.")

            if(A2.x != B2.x):
                r2sl =  (A2.z - B2.z) / (A2.x - B2.x)#slope of r2
                alpha2 = math.degrees(math.atan(r2sl))
                if(not isAcceptable(A2,B2,C,-1)):
                    raise ValueError("Error in knee bend.")
            else:
                alpha2 = -90
                if(not isAcceptable(A2,B2,C,-1)):
                    raise ValueError("Error in knee bend.")

            points = {"B1":B1,"B2":B2,"C":C}
        else:# B1 is the one with bigger x, if x is equal, then with bigger y
            if(B11.x > B12.x):
                B1 = co.Point(B11.x,B11.y,B11.z)
            elif(B12.x > B11.x):
                B1 = co.Point(B12.x,B12.y,B12.z)
            else:
                if(B11.z > B12.z):
                    B1 = co.Point(B11.x,B11.y,B11.z)
                elif(B12.z > B11.z):
                    B1 = co.Point(B12.x,B12.y,B12.z)
                else:
                    raise ValueError("B11==B12!!!!")  
            #ready to calculate alpha 1 and alpha 2
            if(A1.x != B1.x):
                r1sl =  (A1.z - B1.z) / (A1.x - B1.x)#slope of r1
                alpha1 = math.degrees(math.atan(r1sl))
                if(not isAcceptable(A1,B1,C,1)):
                    raise ValueError("Error in knee bend.")
            else:
                alpha1 = -90
                if(not isAcceptable(A1,B1,C,1)):
                    raise ValueError("Error in knee bend.")

            if(A2.x != B2.x):
                r2sl =  (A2.z - B2.z) / (A2.x - B2.x)#slope of r2
                alpha2 = math.degrees(math.atan(r2sl))
                if(not isAcceptable(A2,B2,C,-1)):
                    raise ValueError("Error in knee bend.")
            else:
                alpha2 = -90
                if(not isAcceptable(A2,B2,C,-1)):
                    raise ValueError("Error in knee bend.")

            points = {"B1":B1,"B2":B2,"C":C}
    else:#true B2 is one with x lesser than D.x,i.e. the one further back
        if(B21.x < D.x and B22.x >= D.x):
            B2 = co.Point(B21.x,B21.y,B21.z)
        elif(B22.x < D.x and B21.x >= D.x):
            B2 = co.Point(B22.x,B22.y,B22.z)
        else:
            if(B21.z < B22.z):
                B2 = co.Point(B21.x,B21.y,B21.z)
            elif(B22.z < B21.z):
                B2 = co.Point(B22.x,B22.y,B22.z)
            else:
                raise ValueError("B21==B22!!!!")
        
        xc = (co.p1 * B2.x + co.n2 * D.x)/(co.n2 + co.p1)
        zc = (co.p1 * B2.z + co.n2 * D.z)/(co.n2 + co.p1)
        C = co.Point(xc, 0, zc)

        K3 = co.Circle(C, co.n1)
        K4 = co.Circle(A1, co.r1)
        B11, B12 = K3.intersect(K4)
        if(B11==None and B12==None):
            raise ValueError("C==A1")
        elif(B11!=None and B12==None):
            B1 = co.Point(B11.x,B11.y,B11.z)
            #ready to calculate alpha 1 and alpha 2
            if(A1.x != B1.x):
                r1sl =  (A1.z - B1.z) / (A1.x - B1.x)#slope of r1
                alpha1 = math.degrees(math.atan(r1sl))
                if(not isAcceptable(A1,B1,C,1)):
                    raise ValueError("Error in knee bend.")
            else:
                alpha1 = -90
                if(not isAcceptable(A1,B1,C,1)):
                    raise ValueError("Error in knee bend.")

            if(A2.x != B2.x):
                r2sl =  (A2.z - B2.z) / (A2.x - B2.x)#slope of r2
                alpha2 = math.degrees(math.atan(r2sl))
                if(not isAcceptable(A2,B2,C,-1)):
                    raise ValueError("Error in knee bend.")
            else:
                alpha2 = -90
                if(not isAcceptable(A2,B2,C,-1)):
                    raise ValueError("Error in knee bend.")

            points = {"B1":B1,"B2":B2,"C":C}
        else:# B1 is the one with bigger x, if x is equal, then with bigger y
            if(B11.x > B12.x):
                B1 = co.Point(B11.x,B11.y,B11.z)
            elif(B12.x > B11.x):
                B1 = co.Point(B12.x,B12.y,B12.z)
            else:
                if(B11.z > B12.z):
                    B1 = co.Point(B11.x,B11.y,B11.z)
                elif(B12.z > B11.z):
                    B1 = co.Point(B12.x,B12.y,B12.z)
                else:
                    raise ValueError("B11==B12!!!!")  
            #ready to calculate alpha 1 and alpha 2
            if(A1.x != B1.x):
                r1sl =  (A1.z - B1.z) / (A1.x - B1.x)#slope of r1
                alpha1 = math.degrees(math.atan(r1sl))
                if(not isAcceptable(A1,B1,C,1)):
                    raise ValueError("Error in knee bend.")
            else:
                alpha1 = -90
                if(not isAcceptable(A1,B1,C,1)):
                    raise ValueError("Error in knee bend.")

            if(A2.x != B2.x):
                r2sl =  (A2.z - B2.z) / (A2.x - B2.x)#slope of r2
                alpha2 = math.degrees(math.atan(r2sl))
                if(not isAcceptable(A2,B2,C,-1)):
                    raise ValueError("Error in knee bend.")
            else:
                alpha2 = -90
                if(not isAcceptable(A2,B2,C,-1)):
                    raise ValueError("Error in knee bend.")

            points = {"B1":B1,"B2":B2,"C":C}

    return alpha1,alpha2,points

if __name__=="__main__":
    D = co.Point(-1.32489278167, 0, -12.9563484818)
    a1,a2,points=getAlphas(D)
    print("a1:",a1,", a2:",a2)
    for key in points.keys():
        print(key,":",points[key].x,",",points[key].z)
    
        