#ifndef NON_PARALLEL_UTIL_H
#define NON_PARALLEL_UTIL_H

#include<cmath>
#include <Eigen/Dense>
#define PI 3.141592654f

//CONFIGURATION CONSTANTS
#define R0X -2 //distance of A2 from A1 on x axis
#define R0Z 1 //distance of A2 from A1 on z axis
#define R1 3 //distance from A1 to B1, MAKE IT EQUAL TO r0x?
#define R2 7 //distance from A2 to B2
#define N1 6.5 //distance from B1 to C
#define N2 4 //distance from B2 to C
#define P1 6.5 //distance from C to D

/*class Base{    //unneeded
public:
    double x, y, z;

    Base(double xin, double yin, double zin)
    :x(xin),y(yin),z(zin)
    {}

    //add rotateAxis
};*/

class Point{
    /*A representation of a point with coordinates x, y and z 
        ,angle alpha which was used to get said coordinates, and angle phi that represents rotation around the z axis

        Methods:

        -dist: Returns thistance from this point to another

        -translate: translates this Point to given Point

        -rotateX:rotates around x axis

        -rotateY:rotates around y axis

        -rotateZ:rotates around z axis
    */
public:
    double x, y, z;
    
    Point(double x=nan("1"), double y=nan("1"), double z=nan("1"));

    double dist(Point other);

    void translate(Point toPoint);

    void rotateX(double angle);

    void rotateY(double angle);

    void rotateZ(double angle);

    //add rotateAxis
    void reduce();
};

class Linear{
    /*A representation of a linear function determined from one Point and the sope of the line.
    
    !!!The line is in x|z plane and should be used for Points with y=0!!!
    
        Methods:

        -get_z: solves the linear function for given x
    */
public:
    double intercept;
    double k;
    Point X;

    Linear(Point X, double slope);

    double get_z(double xCoord);
};

class Circle{
    /*A representation of a circle given by an origin Point and the radius
        Methods:

        -intersect(Circle other): returns a pointer to a Point array of no, one or two points of intersection
    
    */
public:
    Point origin;
    double radius;
    
    Circle(Point origin,double radius);

    void intersect(Circle other,Point& first, Point& second);
};

class Pose{
public:
    Point D;
    double alpha1;
    double alpha2;

    Pose(Point D, double alpha1,double alpha2);
};

bool isAcceptable(Point A, Point B, Point C, int direct);

void getAlphas(Point D, double& alpha1, double& alpha2);

Point getD(double alpha1, double alpha2);
#endif