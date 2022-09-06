import math
import numpy as np
import coord_class_3D_p_v2 as co
import paralel_inv_kinematic_v2 as inv

def getD(alpha0, alpha1, alpha2, leg):
    """ Calculates the position of one foot of the specified leg.
    
        alpha0 - angle at shoulder\n
        alpha1 - angle at A1\n
        alpha2 - angle at A2\n
        leg - "l" or "r"
    """

    x_leg_origin = None
    if (leg == 'l'):
        x_leg_origin = co.LEFT_LEG_ORIGIN
    elif (leg == 'r'):
        x_leg_origin = co.RIGHT_LEG_ORIGIN
    else:
        raise ValueError("Invalid leg value.")

    A1 = co.T_A1_LEG.trans() # position of A1
    A2 = A1 + co.A2_IN_A1

    #1 - B1 calculation
    B1 = np.array([A1[0] + co.N1_LEN * math.cos(math.radians(alpha1)),
                   0,
                   A1[2] + co.N1_LEN * math.sin(math.radians(alpha1))])
    B1a = np.array([B1[0] - co.F_LEN, 0, B1[2]])

    A2a = np.array([A2[0] + co.R_LEN * math.cos(math.radians(alpha2)),
                    0,
                    A2[2] + co.R_LEN * math.sin(math.radians(alpha2))])

    #2 - calculating Da
    z = np.linalg.norm(A2a - B1a)
    y = np.linalg.norm(A2a - B1)

    mid = (z**2 + co.L_LEN**2 - co.N2_LEN**2) / (2 * z * co.L_LEN)
    zeta = math.degrees(math.acos(mid))
    theta = math.degrees(math.acos((z**2 + co.F_LEN**2 - y**2) / (2 * z * co.F_LEN)))
    Da = np.array([B1a[0] + co.P_LEN * math.cos(math.radians(zeta + theta + co.PHI)),
                   0,
                   B1a[2] + co.P_LEN * math.sin(math.radians(zeta + theta + co.PHI))])

    #3 - calculating D
    unrot_D = np.array([Da[0] + co.F_LEN, 0, Da[2]])

    #4 - rotating D by alpha0 around x axis (axis of rotation of servo A0, the current origin)
    alpha0rad = math.radians(alpha0)
    s0 = math.sin(alpha0rad)
    c0 = math.cos(alpha0rad)
    rotMat = np.array([[1, 0, 0],
                       [0, c0, -s0],
                       [0, s0, c0]])

    D_not_hom = rotMat.dot(unrot_D) # resulting D is not homogenous

    #5 - trannsforming to ROOT
    D_hom = np.array([D_not_hom[0],D_not_hom[1],D_not_hom[2],1])

    D = x_leg_origin.matrix.dot(D_hom)

    # 6 - checking inverse model
    al1 = alpha1
    al2 = alpha2
    al0 = alpha0

    chk0,chk1,chk2,_ = inv.getAlphas(D,leg)
    if(math.fabs(chk1 - al1) > 0.1 or math.fabs(chk2 - al2) > 0.1 or math.fabs(chk0 - al0) > 0.1):
        raise ValueError("Not consistent with inverse model.")

    return D

if __name__=="__main__":
    alpha0 = float(input("0: "))
    alpha1 = float(input("1: "))
    alpha2 = float(input("2: "))

    D = getD(alpha0,alpha1,alpha2,"r")
    D = co.RIGHT_LEG_ORIGIN.inv().matrix.dot(D)

    print(D)