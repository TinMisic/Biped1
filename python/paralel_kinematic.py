import math
import coord_class_3D_p as co
import paralel_inv_kinematic as inv
import matplotlib.pyplot as plt


A1 = co.Point(0,0,0)
A2 = co.Point(A1.x + co.r0x, 0, A1.z + co.r0z)


def getD(alpha1,alpha2):
    #1-getting B1
    B1 = co.Point(A1.x + co.n1 * math.cos(math.radians(alpha1)), 0, A1.z + co.n1 * math.sin(math.radians(alpha1)))
    B1a = co.Point(B1.x - co.f, 0, B1.z)

    A2a = co.Point(A2.x + co.r * math.cos(math.radians(alpha2)),0 ,A2.z + co.r * math.sin(math.radians(alpha2)))

    #2-getting Da
    z = A2a.dist(B1a)
    y = A2a.dist(B1)

    mid = (z**2 + co.l**2 - co.n2**2) / (2 * z * co.l)
    zeta = math.degrees(math.acos(mid))
    theta = math.degrees(math.acos((z**2 + co.f**2 - y**2) / (2 * z * co.f)))
    xda = B1a.x + co.p * math.cos(math.radians(zeta + theta + co.phi))
    zda = B1a.z + co.p * math.sin(math.radians(zeta + theta + co.phi))
    Da = co.Point(xda, 0, zda)
    #3-getting D
    D = co.Point(Da.x + co.f,0,Da.z)

    #4-checking with the inverse model
    al1 = alpha1
    al2 = alpha2
    if(alpha1 < -90):
        al1 = alpha1 + 180
    if(alpha2 < -90):
        al2 = alpha2 + 180
    
    chk1,chk2,_ = inv.getAlphas(D)
    if(math.fabs(chk1 - al1) > 0.1 or math.fabs(chk2 - al2) > 0.1):
        raise ValueError("Not consistent with inverse model.")

    return D

def main():
    counter = 0
    success = 0

    x=list()
    y=list()

    plt.scatter([A1.x,A2.x],[A1.z,A2.z],s=5,c="red")

    for alpha1db in range(0,-361,-1):
        alpha1 = alpha1db * 1.0 / 2
        for alpha2db in range(-240,-601,-1): #POPRAVI KUTEVE
            alpha2 = alpha2db * 1.0 / 2
            counter+=1
            print("a1:",alpha1,", a2:",alpha2," ",end="")
            try:
                D = getD(alpha1,alpha2)
            except ValueError as err:
                print("NO: ",err)
                D = None
            if(D != None):
                success+=1
                print("YES")
                x.append(D.x)
                y.append(D.z)

                
    print("success rate:",success/counter*100,"%")
    plt.axis("scaled")
    plt.xlim(-20, 20)
    plt.ylim(-20, 20)
    plt.scatter(x,y,s=3,c="#05ff55")
    plt.show()
    input("Press any key...")

if __name__=="__main__":
    main()