import math
import numpy as np
import coord_class_3D_p_v2 as co
import matplotlib.pyplot as plt

def isAcceptable(A, B, C, direct):
    '''direct: left is -1, right is 1'''
    vec1 = np.array([A[0] - B[0], 0, A[2] - B[2]])
    vec2 = np.array([C[0] - B[0], 0, C[2] - B[2]])

    angle = 90
    if(vec1[0] != 0):
        angle = math.degrees(math.atan2(vec1[2], vec1[0]))
        if(angle < 0):
            angle += 180

    # rotate around Y
    phi = math.radians(90 - angle)
    operator=np.array([[math.cos(phi),0,-math.sin(phi)],[0,1,0],[math.sin(phi),0,math.cos(phi)]])
    point_arr=np.array([vec2[0],vec2[1],vec2[2]])

    vec2=np.matmul(operator,point_arr)
    if((direct == 1 and vec2[0]<= 0) or (direct == -1 and vec2[0] >= 0)):
        return True
    return False

def getAlphas(D, leg):
    """Calculates the servo angles for the given end-effector position and leg ('l' or 'r').
        @raises ValueError
    """
    
    #1 - initialization
    alpha0 = 0
    alpha1 = 0
    alpha2 = 0
    points = dict()
    D = np.array([D[0],D[1],D[2],1]) 

    x_leg_origin = None
    if (leg == 'l'):
        x_leg_origin = co.LEFT_LEG_ORIGIN
    elif (leg == 'r'):
        x_leg_origin = co.RIGHT_LEG_ORIGIN
    else:
        raise ValueError("Invalid leg value.")

    #2 - transform D from ROOT to x_leg_origin
    D_l = x_leg_origin.inv().matrix.dot(D)

    #3 - unrotate D to find alpha0
    alpha0 = math.degrees(math.atan2(D_l[2], D_l[1])) + 90
    phi0 = math.radians(alpha0)
    rotX = np.array([[1,0,0,0],[0,math.cos(phi0),-math.sin(phi0),0],[0,math.sin(phi0),math.cos(phi0),0],[0, 0, 0, 1]])
    
    D_unrot = rotX.dot(D_l) # now D is on plane x|z
    D_fin = D_unrot[0:3] # de-homogenize
    points["D"] = D_fin
    points["Da"] = np.array([D_fin[0]-co.F_LEN,D_fin[1],D_fin[2]])

    A1 = co.T_A1_LEG.trans() # position of A1 in leg frame
    A2 = A1 + co.A2_IN_A1
    points["A1"]=A1
    points["A2"]=A2
    points["A1a"] = np.array([A1[0]-co.F_LEN,A1[1],A1[2]])

    if(np.linalg.norm(A1 - D_fin) > co.N1_LEN + co.P_LEN):
        raise ValueError("D is unreachable.")

    #3 - find B1
    K1 = co.Circle(A1, co.N1_LEN)
    K2 = co.Circle(D_fin, co.P_LEN)
    B11, B12 = K1.intersect(K2)
    if(B11 is None and B12 is None):
        raise ValueError("D must not be equal to A1.")
    elif(B11 is not None and B12 is None):
        B1 = np.array([B11[0],B11[1],B11[2]])
        points["B1"]=B1
        points["B1a"] = np.array([B1[0]-co.F_LEN,B1[1],B1[2]])
        if(A1[0] != B1[0]):
            sl_n1 = (A1[2] - B1[2]) / (A1[0] - B1[0])
            alpha1 = math.degrees(math.atan2((B1[2] - A1[2]),(B1[0] - A1[0]))) 
        else:
            alpha1 = -90

        if(D_fin[0] != B1[0]):
            sl_p = (D_fin[2] - B1[2]) / (D_fin[0] - B1[0])
            beta1 = math.degrees(math.atan(sl_p))
        else:
            beta1 = -90

        xc = B1[0] - co.F_LEN + co.L_LEN * math.cos(math.radians(beta1 - co.PHI))
        zc = B1[2] + co.L_LEN * math.sin(math.radians(beta1 - co.PHI))
        C = np.array([xc, 0, zc])
        points["C"]=C

        K3 = co.Circle(A2, co.R_LEN)
        K4 = co.Circle(C, co.N2_LEN)

        A2a1, A2a2 = K3.intersect(K4)

        if(A2a1 is None and A2a2 is None):
            raise ValueError("Error in n2")
        elif(A2a1 is not None and A2a2 is None):
            A2a = np.array([A2a1[0], A2a1[1], A2a1[2]])
            points["A2a"]=A2a

            if(A2a[0] != A2[0]):
                sl_r = (A2[2] - A2a[2]) / (A2[0] - A2a[0])
                alpha2 = math.degrees(math.atan2((A2a[2] - A2[2]), (A2a[0] - A2a[0]))) 
            else:
                alpha2 = -90

        else:
            if(isAcceptable(A2, A2a1, C,-1)):
                A2a = np.array([A2a1[0],A2a1[1],A2a1[2]])
            else:
                A2a = np.array([A2a2[0],A2a2[1],A2a2[2]])

            points["A2a"]=A2a
            if(A2a[0] != A2[0]):
                sl_r = (A2[2] - A2a[2]) / (A2[0] - A2a[0])
                alpha2 = math.degrees(math.atan2((A2a[2] - A2[2]),(A2a[0] - A2[0]))) 
            else:
                alpha2 = -90

    else:
        if(isAcceptable(A1,B11,D_fin,-1)):
            B1 = np.array([B11[0], B11[1], B11[2]])
        else:
            B1 = np.array([B12[0], B12[1], B12[2]])

        points["B1"]=B1
        points["B1a"] = np.array([B1[0]-co.F_LEN,B1[1],B1[2]])
        if(A1[0] != B1[0]):
            sl_n1 = (A1[2] - B1[2]) / (A1[0] - B1[0])
            alpha1 = math.degrees(math.atan2((B1[2] - A1[2]), (B1[0] - A1[0])))
        else:
            alpha1 = -90

        if(D_fin[0] != B1[0]):
            sl_p = (D_fin[2] - B1[2]) / (D_fin[0] - B1[0])
            beta1 = math.degrees(math.atan(sl_p))
        else:
            beta1 = -90

        xc = B1[0] - co.F_LEN + co.L_LEN * math.cos(math.radians(beta1 - co.PHI))
        zc = B1[2] + co.L_LEN * math.sin(math.radians(beta1 - co.PHI))
        C =  np.array([xc, 0, zc])
        points["C"]=C

        K3 = co.Circle(A2, co.R_LEN)
        K4 = co.Circle(C, co.N2_LEN)

        A2a1, A2a2 = K3.intersect(K4)

        if(A2a1 is None and A2a2 is None):
            raise ValueError("Error in n2")
        elif(A2a1 is not None and A2a2 is None):
            A2a = np.array([A2a1[0], A2a1[1], A2a1[2]])
            points["A2a"]=A2a

            if(A2a[0] != A2[0]):
                sl_r = (A2[2] - A2a[2]) / (A2[0] - A2a[0])
                alpha2 = math.degrees(math.atan2((A2a[2] - A2[2]), (A2a[0] - A2[0]))) 
            else:
                alpha2 = -90

        else:
            if(isAcceptable(A2, A2a1, C,-1)):
                A2a = np.array([A2a1[0],A2a1[1],A2a1[2]])
            else:
                A2a = np.array([A2a2[0],A2a2[1],A2a2[2]])

            points["A2a"]=A2a
            if(A2a[0] != A2[0]):
                sl_r = (A2[2] - A2a[2]) / (A2[0] - A2a[0])
                alpha2 = math.degrees(math.atan2((A2a[2] - A2[2]), (A2a[0] - A2[0]))) 
            else:
                alpha2 = -90

    if (alpha2 >= 90 and alpha2 <=180):
        alpha2 += -360     #More convenient to have alpha2 always negative
    return alpha0, alpha1, alpha2, points

if __name__=="__main__":
    # x = float(input("x: "))
    # y = float(input("y: "))
    # z = float(input("z: "))

    # D = ([x, y, z, 1])
    D = np.array([217.256889178,0,-401.112638067,1])
    D = co.RIGHT_LEG_ORIGIN.matrix.dot(D)

    a0,a1,a2,pnts=getAlphas(D, "r")
    print("a0: "+str(a0)+"\na1: "+str(a1)+"\na2: "+str(a2))
    xs=list()
    ys=list()
    for k in pnts.keys():
        #print(k + ": " + str(pnts[k][0]) + " , " + str(pnts[k][2]))
        xs.append(pnts[k][0])
        ys.append(pnts[k][2])

    plt.scatter(xs,ys)
    plt.show()
