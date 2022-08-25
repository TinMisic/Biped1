#ifndef PARALLEL_UTIL_H
#define PARALLEL_UTIL_H

#include<cmath>
#include <Eigen/Dense>
#define PI 3.141592654f

//CONFIGURATION CONSTANTS
#define R0X -2.0 //distance of A2 from A1 on x axis
#define R0Z 1.0 //distance of A2 from A1 on z axis
#define R 1.5 //pivot length of servo 2
#define N1 6.5 //length of upper paralelogram
#define N2 7.0 //length of arm from A2a to C
#define F 2.0 //length of foot 
#define L 1.5 //length of elbow at C
#define P 6.5 //distance from C to D
#define PHI 120.0 //angle between elbow and lower leg

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