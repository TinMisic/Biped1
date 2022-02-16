import math
import coord_class_3D as co
import nonparalel_inv_kinematic as inv
import matplotlib.pyplot as plt


A1 = co.Point(0,0,0)
A2 = co.Point(A1.x + co.r0x, 0, A1.z + co.r0z)


def getD(alpha1,alpha2):
    #1-getting Bs
    B1 = co.Point(A1.x + co.r1 * math.cos(math.radians(alpha1)), 0, A1.z + co.r1 * math.sin(math.radians(alpha1)))
    B2 = co.Point(A2.x + co.r2 * math.cos(math.radians(alpha2)), 0, A2.z + co.r2 * math.sin(math.radians(alpha2)))

    #2-getting C
    t1sl = (B2.z - B1.z) / (B2.x - B1.x) #slope of imaginary line t1 between B1 and B2
    t1 = co.Linear(B1, t1sl) #t1 linear representation
    t1len = B1.dist(B2) #length of t1

    beta1 = math.degrees(math.acos(((t1len**2) + (co.n1**2) - (co.n2**2)) / (2 * t1len * co.n1)))
    beta2 = math.degrees(math.acos(((t1len**2) + (co.n2**2) - (co.n1**2)) / (2 * t1len * co.n2)))

    C = co.Point(0,0,0)
    C.x = B1.x + co.n1 * math.cos(math.radians(180 + math.degrees(math.atan(t1sl)) + beta1))
    C.z = B1.z + co.n1 * math.sin(math.radians(180 + math.degrees(math.atan(t1sl)) + beta1))

    #3-getting D
    D = co.Point(0,0,0)
    D.x = C.x + co.p1 * math.cos(math.radians(math.degrees(math.atan(t1sl)) - beta2))
    D.z = C.z + co.p1 * math.sin(math.radians(math.degrees(math.atan(t1sl)) - beta2))

    #4-checking with the inerse model
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
        for alpha2db in range(0,-361,-1):
            alpha2 = alpha2db * 1.0 / 2
            counter+=1
            print("a1:",alpha1,", a2:",alpha2," ",end="")
            try:
                D = getD(alpha1,alpha2)
            except:
                print("NO")
                D = None
            if(D != None):
                success+=1
                print("YES")
                x.append(D.x)
                y.append(D.z)

                
    print("success rate:",success/counter*100,"%")
    plt.axis("scaled")
    plt.xlim(-12, 12)
    plt.ylim(-12, 12)
    plt.scatter(x,y,s=3,c="#05ff55")
    plt.show()
    input("Press any key...")

if __name__=="__main__":
    main()