#include "non_parallel_util.h"
 
using Eigen::MatrixXd;

Point::Point(double x, double y, double z)
    :x(x),y(y),z(z)
    {}

double Point::dist(Point other){
        return sqrt(pow(this->x - other.x, 2) + pow(this->y - other.y, 2) + pow(this->z - other.z, 2));
    }

void Point::translate(Point toPoint){
        this->x += toPoint.x;
        this->y += toPoint.y;
        this->z += toPoint.z;
    }

void Point::rotateX(double angle){
        /*Rotates around x axis*/
        double phi = angle * PI / 180.0;
        MatrixXd oper(3,3);
        oper(0,0) = 1;
        oper(0,1) = 0;
        oper(0,2) = 0;
        oper(1,0) = 0;
        oper(1,1) = cos(phi);
        oper(1,2) = -sin(phi);
        oper(2,0) = 0;
        oper(2,1) = sin(phi);
        oper(2,2) = cos(phi);

        MatrixXd point_arr(3,1);
        point_arr(0,0) = this->x;
        point_arr(1,0) = this->y;
        point_arr(2,0) = this->z;

        point_arr = oper * point_arr;
        this->x = point_arr(0,0);
        this->y = point_arr(1,0);
        this->z = point_arr(2,0);
    }

void Point::rotateY(double angle){
        /*Rotates around y axis*/
        double phi = angle * PI / 180.0;
        MatrixXd oper(3,3);
        oper(0,0) = cos(phi);
        oper(0,1) = 0;
        oper(0,2) = -sin(phi);
        oper(1,0) = 0;
        oper(1,1) = 1;
        oper(1,2) = 0;
        oper(2,0) = sin(phi);
        oper(2,1) = 0;
        oper(2,2) = cos(phi);

        MatrixXd point_arr(3,1);
        point_arr(0,0) = this->x;
        point_arr(1,0) = this->y;
        point_arr(2,0) = this->z;

        point_arr = oper * point_arr;
        this->x = point_arr(0,0);
        this->y = point_arr(1,0);
        this->z = point_arr(2,0);
    }

void Point::rotateZ(double angle){
        /*Rotates around z axis*/
        double phi = angle * PI / 180.0;
        MatrixXd oper(3,3);
        oper(0,0) = cos(phi);
        oper(0,1) = -sin(phi);
        oper(0,2) = 0;
        oper(1,0) = sin(phi);
        oper(1,1) = cos(phi);
        oper(1,2) = 0;
        oper(2,0) = 0;
        oper(2,1) = 0;
        oper(2,2) = 1;

        MatrixXd point_arr(3,1);
        point_arr(0,0) = this->x;
        point_arr(1,0) = this->y;
        point_arr(2,0) = this->z;

        point_arr = oper * point_arr;
        this->x = point_arr(0,0);
        this->y = point_arr(1,0);
        this->z = point_arr(2,0);
    }

void Point::reduce(){
        if (fabs(this->x) <= 1e-6)
            this->x = 0;
        if (fabs(this->y) <= 1e-6)
            this->y = 0;
        if (fabs(this->z) <= 1e-6)
            this->z = 0;
    }

Linear::Linear(Point X, double slope)
    :X(X),k(slope)
    {
        this->intercept = -(this->k * X.x) + X.z;
    }

double Linear::get_z(double xCoord){
        //the actual Linear function, returns Point on line with given x coordinate
        return this->k * xCoord + this->intercept;
    }

Circle::Circle(Point origin,double radius)
    :origin(origin), radius(radius)
    {}

void Circle::intersect(Circle other,Point& first, Point& second){
        if(this->origin.z != other.origin.z){
            double k = - (this->origin.x - other.origin.x) / (this->origin.z - other.origin.z);
            double l = ((pow(this->radius, 2) - pow(other.radius, 2)) - (pow(this->origin.x, 2) - pow(other.origin.x, 2)) - (pow(this->origin.z, 2) - pow(other.origin.z, 2)))/(-2 * (this->origin.z - other.origin.z));
            double a = k*k + 1;
            double b = 2 * k * (l - this->origin.z) - 2 * this->origin.x;
            double c = pow(this->origin.x, 2) - pow(this->radius, 2) + pow((l - this->origin.z), 2);

            double x1 = (-b + sqrt(b*b - 4 * a * c))/(2 * a);
            double x2 = (-b - sqrt(b*b - 4 * a * c))/(2 * a);

            if(x1 == x2){
                first.x = x1;
                first.y = 0;
                first.z = k * x1 + l; //only one point of intersection
            }
            else{
                first.x = x1;
                first.y = 0;
                first.z = k * x1 + l;
                //----//
                second.x = x2;
                second.y = 0;
                second.z = k * x2 + l;
            }
        }
        else if (this->origin.x != other.origin.x){
            double k = - (this->origin.z - other.origin.z)/(this->origin.x - other.origin.x);
            double l = ((pow(this->radius, 2) - pow(other.radius, 2)) - (pow(this->origin.x, 2) - pow(other.origin.x, 2)) - (pow(this->origin.z, 2) - pow(other.origin.z, 2)))/(-2 * (this->origin.x - other.origin.x));
            double a = k*k + 1;
            double b = 2 * k * (l - this->origin.x) - 2 * this->origin.z;
            double c = pow(this->origin.z, 2) - pow(this->radius, 2) + pow((l - this->origin.x), 2);

            double z1 = (-b + sqrt(b*b - 4 * a * c))/(2 * a);
            double z2 = (-b - sqrt(b*b - 4 * a * c))/(2 * a);
            if(z1 == z2){
                first.x = k * z1 + l;
                first.y = 0;
                first.z = z1;
                //only one point
            }
            else{
                first.x = k * z1 + l;
                first.y = 0;
                first.z = z1;
                //----//
                second.x = k * z2 + l;
                second.y = 0;
                second.z = z2;
            }
        }
        else{
            ;//no points or infinite points of intersection, circles have same origin
        }
    }

Pose::Pose(Point D, double alpha1,double alpha2)
    :D(D),alpha1(alpha1),alpha2(alpha2)
    {}

Point A1 = Point(0,0,0);
Point A2 = Point(A1.x + R0X, 0, A1.z + R0Z);

bool isAcceptable(Point A, Point B, Point C, int direct)
{   //direct: left is -1, right is 1
    Point vec1 = Point(A.x - B.x, 0, A.z - B.z);
    Point vec2 = Point(C.x - B.x, 0, C.z - B.z);

    double angle = 90.0;
    if(vec1.x != 0){
        angle = atan(vec1.z / vec1.x) * 180 / PI;
        if(angle < 0)
            angle += 180;
    }

    vec2.rotateY(90 - angle);
    if((direct == 1 && vec2.x <= 0) || (direct == -1 && vec2.x >= 0))
        return true;
    return false;
}

void getAlphas(Point D, double& alpha1, double& alpha2)
{
    if(A2.dist(D) > R2 + N2 + P1)
        throw "D is unreachable";
    Circle K1 = Circle(A2, R2);
    Circle K2 = Circle(D, N2 + P1);
    Point B21, B22;
    K1.intersect(K2, B21, B22);
    if(std::isnan(B21.x) && std::isnan(B22.x))
        throw "D must not be equal to A1 or A2.";
    else if(!std::isnan(B21.x) && std::isnan(B22.x)){
        Point B2 = Point(B21.x, B21.y, B21.z);
        double xc = (P1 * B2.x + N2 * D.x) / (N2 + P1);
        double zc = (P1 * B2.z + N2 * D.z) / (N2 + P1);
        Point C = Point(xc, 0, zc);

        Circle K3 = Circle(C, N1);
        Circle K4 = Circle(A1, R1);
        Point B11, B12;
        K3.intersect(K4, B11, B12);
        if(std::isnan(B11.x) && std::isnan(B12.x))
            throw "C==A1";
        else if(!std::isnan(B11.x) && std::isnan(B12.x)){
            Point B1 = Point(B11.x, B11.y, B11.z);

            if(A1.x != B1.x){
                double r1sl = (A1.z - B1.z) / (A1.x - B1.x);//slope of r1
                alpha1 = 180 / PI * atan(r1sl);
                if(!isAcceptable(A1,B1,C,1))
                    throw "Error in knee bend";
            }
            else{
                alpha1 = -90;
                if(!isAcceptable(A1,B1,C,1))
                    throw "Error in knee bend";
            }

            if(A2.x != B2.x){
                double r2sl =  (A2.z - B2.z) / (A2.x - B2.x);//slope of r2
                alpha2 = 180 / PI * atan(r2sl);
                if(!isAcceptable(A2,B2,C,-1))
                    throw "Error in knee bend.";
            }
            else{
                alpha2 = -90;
                if(!isAcceptable(A2,B2,C,-1))
                    throw "Error in knee bend.";
            }
        }
        else{
            Point B1;
            if(B11.x > B12.x){
                B1.x = B11.x;
                B1.y = B11.y;
                B1.z = B11.z;
            }
            else if(B12.x > B11.x){
                B1.x = B12.x;
                B1.y = B12.y;
                B1.z = B12.z;
            }
            else{
                if(B11.z > B12.z){
                    B1.x = B11.x;
                    B1.y = B11.y;
                    B1.z = B11.z;
                }
                else if(B12.z > B11.z){
                    B1.x = B12.x;
                    B1.y = B12.y;
                    B1.z = B12.z;
                }
                else
                    throw "B11==B12";
            }

            if(A1.x != B1.x){
                double r1sl = (A1.z - B1.z) / (A1.x - B1.x);//slope of r1
                alpha1 = 180 / PI * atan(r1sl);
                if(!isAcceptable(A1,B1,C,1))
                    throw "Error in knee bend";
            }
            else{
                alpha1 = -90;
                if(!isAcceptable(A1,B1,C,1))
                    throw "Error in knee bend";
            }

            if(A2.x != B2.x){
                double r2sl =  (A2.z - B2.z) / (A2.x - B2.x);//slope of r2
                alpha2 = 180 / PI * atan(r2sl);
                if(!isAcceptable(A2,B2,C,-1))
                    throw "Error in knee bend.";
            }
            else{
                alpha2 = -90;
                if(!isAcceptable(A2,B2,C,-1))
                    throw "Error in knee bend.";
            }
        }
    }
    else{
        Point B2;
        if(B21.x < D.x && B22.x >= D.x){
            B2.x = B21.x;
            B2.y = B21.y;
            B2.z = B21.z;
        }
        else if(B22.x < D.x && B21.x >= D.x){
            B2.x = B22.x;
            B2.y = B22.y;
            B2.z = B22.z;
        }
        else{
            if(B21.z < B22.z){
                B2.x = B21.x;
                B2.y = B21.y;
                B2.z = B21.z;
            }
            else if(B22.z < B21.z){
                B2.x = B22.x;
                B2.y = B22.y;
                B2.z = B22.z;
            }
            else
                throw "B21==B22";
        }

        double xc = (P1 * B2.x + N2 * D.x) / (N2 + P1);
        double zc = (P1 * B2.z + N2 * D.z) / (N2 + P1);
        Point C = Point(xc, 0, zc);

        Circle K3 = Circle(C, N1);
        Circle K4 = Circle(A1, R1);
        Point B11, B12;
        K3.intersect(K4, B11, B12);
        if(std::isnan(B11.x) && std::isnan(B12.x))
            throw "C==A1";
        else if(!std::isnan(B11.x) && std::isnan(B12.x)){
            Point B1 = Point(B11.x, B11.y, B11.z);

            if(A1.x != B1.x){
                double r1sl = (A1.z - B1.z) / (A1.x - B1.x);//slope of r1
                alpha1 = 180 / PI * atan(r1sl);
                if(!isAcceptable(A1,B1,C,1))
                    throw "Error in knee bend";
            }
            else{
                alpha1 = -90;
                if(!isAcceptable(A1,B1,C,1))
                    throw "Error in knee bend";
            }

            if(A2.x != B2.x){
                double r2sl =  (A2.z - B2.z) / (A2.x - B2.x);//slope of r2
                alpha2 = 180 / PI * atan(r2sl);
                if(!isAcceptable(A2,B2,C,-1))
                    throw "Error in knee bend.";
            }
            else{
                alpha2 = -90;
                if(!isAcceptable(A2,B2,C,-1))
                    throw "Error in knee bend.";
            }
        }
        else{
            Point B1;
            if(B11.x > B12.x){
                B1.x = B11.x;
                B1.y = B11.y;
                B1.z = B11.z;
            }
            else if(B12.x > B11.x){
                B1.x = B12.x;
                B1.y = B12.y;
                B1.z = B12.z;
            }
            else{
                if(B11.z > B12.z){
                    B1.x = B11.x;
                    B1.y = B11.y;
                    B1.z = B11.z;
                }
                else if(B12.z > B11.z){
                    B1.x = B12.x;
                    B1.y = B12.y;
                    B1.z = B12.z;
                }
                else
                    throw "B11==B12";
            }

            if(A1.x != B1.x){
                double r1sl = (A1.z - B1.z) / (A1.x - B1.x);//slope of r1
                alpha1 = 180 / PI * atan(r1sl);
                if(!isAcceptable(A1,B1,C,1))
                    throw "Error in knee bend";
            }
            else{
                alpha1 = -90;
                if(!isAcceptable(A1,B1,C,1))
                    throw "Error in knee bend";
            }

            if(A2.x != B2.x){
                double r2sl =  (A2.z - B2.z) / (A2.x - B2.x);//slope of r2
                alpha2 = 180 / PI * atan(r2sl);
                if(!isAcceptable(A2,B2,C,-1))
                    throw "Error in knee bend.";
            }
            else{
                alpha2 = -90;
                if(!isAcceptable(A2,B2,C,-1))
                    throw "Error in knee bend.";
            }
        }
    }
}

Point getD(double alpha1, double alpha2)
{
    Point B1 = Point(A1.x + R1 * cos(PI/180 * alpha1), 0, A1.z + R1 * sin(PI/180 * alpha1));
    Point B2 = Point(A2.x + R2 * cos(PI/180 * alpha2), 0, A2.z + R2 * sin(PI/180 * alpha2));
    
    double t1sl = (B2.z - B1.z) / (B2.x - B1.x);
    Linear t1 = Linear(B1, t1sl);
    double t1len = B1.dist(B2);

    double beta1 = 180.0/PI * (acos(((t1len*t1len) + (N1*N1) - (N2*N2)) / (2 * t1len * N1)));
    double beta2 = 180.0/PI * (acos(((t1len*t1len) + (N2*N2) - (N1*N1)) / (2 * t1len * N2)));

    Point C = Point(0,0,0);
    C.x = B1.x + N1 * cos(PI/180 * (180 + 180.0/PI * (atan(t1sl)) + beta1));
    C.z = B1.z + N1 * sin(PI/180 * (180 + 180.0/PI * (atan(t1sl)) + beta1));

    Point D = Point(0,0,0);
    D.x = C.x + P1 * cos(PI/180 * (180.0/PI * (atan(t1sl)) - beta2));
    D.z = C.z + P1 * sin(PI/180 * (180.0/PI * (atan(t1sl)) - beta2));

    double al1 = alpha1, al2 = alpha2;
    if(alpha1 < -90)
        al1 = alpha1 + 180;
    if(alpha2 < -90)
        al1 = alpha1 + 180;

    double chk1 = 0, chk2 = 0;
    getAlphas(D, chk1, chk2);
    if(fabs(chk1 - al1) > 0.1 || fabs(chk2 - al2) > 0.1)
        throw "Not consistent with inverse model";
    
    return D;
}