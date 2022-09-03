import math
import numpy as np
import coord_class_3D_p_v2 as co

def isAcceptable(A, B, C, direct):
    '''direct: left is -1, right is 1'''
    vec1 = np.array([A[0] - B[0], 0, A[2] - B[2]])
    vec2 = np.array([C[0] - B[0], 0, C[2] - B[2]])

    angle = 90
    if(vec1[0] != 0):
        angle = math.degrees(math.atan(vec1[2] / vec1[0]))
        if(angle < 90):
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
    angle0 = math.degrees(math.atan2(D_l[2], D_l[1])) + 90
    phi0 = math.radians(angle0)
    rotX = np.array([[1,0,0,0],[0,math.cos(phi0),-math.sin(phi0),0],[0,math.sin(phi0),math.cos(phi0),0],[0, 0, 0, 1]])
    
    D_unrot = rotX.dot(D_l) # now D is on plane x|z
    D_fin = D_unrot[0:3] # de-homogenize

    A1 = co.T_A1_LEG.trans() # position of A1 in leg frame
    A2 = A1 + co.A2_IN_A1

    if(np.linalg.norm(A1 - D_fin) > co.N1_LEN + co.P_LEN):
        raise ValueError("D is unreachable.")

    #3 - find B1
    K1 = co.Circle(A1, co.N1_LEN)
    K2 = co.Circle(D_fin, co.P_LEN)
    B11, B12 = K1.intersect(K2)
    if(B11 == None and B12 == None):
        raise ValueError("D must not be equal to A1.")
    elif(B11!=None and B12==None):
        B1 = co.Point(B11[0],B11[1],B11[2])
        if(A1[0] != B1[0]):
            sl_n1 = (A1[2] - B1[2]) / (A1[0] - B1[0])
            alpha1 = math.degrees(math.atan(sl_n1)) 
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

        K3 = co.Circle(A2, co.R_LEN)
        K4 = co.Circle(C, co.N2_LEN)

        A2a1, A2a2 = K3.intersect(K4)

        if(A2a1==None and A2a2==None):
            raise ValueError("Error in n2")
        elif(A2a1!=None and A2a2==None):
            A2a = np.array([A2a1[0], A2a1[1], A2a1[2]])

            if(A2a[0] != A2[0]):
                sl_r = (A2[2] - A2a[2]) / (A2[0] - A2a[0])
                alpha2 = math.degrees(math.atan(sl_r)) 
            else:
                alpha2 = -90

            points["B1"] = B1
            points["C"] = C
            points["A2a"] = A2a
        else:
            if(isAcceptable(A2, A2a1, C,-1)):
                A2a = np.array([A2a1[0],A2a1[1],A2a1[2]])
            else:
                A2a = np.array([A2a2[0],A2a2[1],A2a2[2]])

            if(A2a[0] != A2[0]):
                sl_r = (A2[2] - A2a[2]) / (A2[0] - A2a[0])
                alpha2 = math.degrees(math.atan(sl_r)) 
            else:
                alpha2 = -90

            points["B1"] = B1
            points["C"] = C
            points["A2a"] = A2a
    else:
        if(isAcceptable(A1,B11,D_fin,-1)):
            B1 = np.array([B11[0], B11[1], B11[2]])
        else:
            B1 = np.array([B12[0], B12[1], B12[2]])

        if(A1[0] != B1[0]):
            sl_n1 = (A1[2] - B1[2]) / (A1[0] - B1[0])
            alpha1 = math.degrees(math.atan(sl_n1))
        else:
            alpha = -90

        if(D_fin[0] != B1[0]):
            sl_p = (D_fin[2] - B1[2]) / (D_fin[0] - B1[0])
            beta1 = math.degrees(math.atan(sl_p))
        else:
            beta1 = -90

        xc = B1[0] - co.F_LEN + co.L_LEN * math.cos(math.radians(beta1 - co.PHI))
        zc = B1[2] + co.L_LEN * math.sin(math.radians(beta1 - co.PHI))
        C =  np.array([xc, 0, zc])

        K3 = co.Circle(A2, co.R_LEN)
        K4 = co.Circle(C, co.N2_LEN)

        A2a1, A2a2 = K3.intersect(K4)

        if(A2a1==None and A2a2==None):
            raise ValueError("Error in n2")
        elif(A2a1!=None and A2a2==None):
            A2a = np.array([A2a1[0], A2a1[1], A2a1[2]])

            if(A2a[0] != A2[0]):
                sl_r = (A2[2] - A2a[2]) / (A2[0] - A2a[0])
                alpha2 = math.degrees(math.atan(sl_r)) 
            else:
                alpha2 = -90

            points["B1"] = B1
            points["C"] = C
            points["A2a"] = A2a
        else:
            if(isAcceptable(A2, A2a1, C,-1)):
                A2a = np.array([A2a1[0],A2a1[1],A2a1[2]])
            else:
                A2a = np.array(A2a2[0],A2a2[1],A2a2[2])

            if(A2a[0] != A2[0]):
                sl_r = (A2[2] - A2a[2]) / (A2[0] - A2a[0])
                alpha2 = math.degrees(math.atan(sl_r)) 
            else:
                alpha2 = -90

            points["B1"] = B1
            points["C"] = C
            points["A2a"] = A2a

    return alpha0, alpha1, alpha2, points