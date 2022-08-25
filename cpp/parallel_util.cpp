#include "parallel_util.h"
 
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
    if(A1.dist(D) > N1 + P)
        throw "D is unreachable.";
    
    Circle K1 = Circle(A1, N1);
    Circle K2 = Circle(D, P);
    Point B11 = Point(), B12 = Point();
    K1.intersect(K2, B11, B12);

    if(std::isnan(B11.x) && std::isnan(B12.x))
        throw "D must not be equal to A1";
    else if(!std::isnan(B11.x) && std::isnan(B12.x)){
        Point B1 = Point(B11.x, B11.y, B11.z);
        if(A1.x != B1.x){
            double sl_n1 = (A1.z - B1.z) / (A1.x - B1.x);
            alpha1 = 180.0 * atan(sl_n1) / PI;
        }
        else
            alpha1 = -90;

        double beta1;
        if(D.x != B1.x){
            double sl_p = (D.z - B1.z) / (D.x - B1.x);
            beta1 = 180.0 * atan(sl_p) / PI;
        }
        else
            beta1 = -90;

        double xc = B1.x - F + L * cos(PI / 180.0 *(beta1 - PHI));
        double zc = B1.z + L * sin(PI / 180.0 *(beta1 - PHI));
        Point C = Point(xc, 0, zc);

        Circle K3 = Circle(A2, R);
        Circle K4 = Circle(C, N2);

        Point A2a1 = Point(), A2a2 = Point();
        K3.intersect(K4, A2a1, A2a2);

        if(std::isnan(A2a1.x) && std::isnan(A2a2.x))
            throw "Error in n2";
        else if(!std::isnan(A2a1.x) && std::isnan(A2a2.x)){
            Point A2a = Point(A2a1.x, A2a1.y, A2a1.z);

            if(A2a.x != A2.x){
                double sl_r = (A2.z - A2a.z) / (A2.x - A2a.x);
                alpha2 = 180.0 * atan(sl_r) / PI;
            }
            else
                alpha2 = -90;
        }
        else{
            Point A2a = Point();
            if(isAcceptable(A2, A2a1, C, -1)){
                A2a.x = A2a1.x;
                A2a.y = A2a1.y;
                A2a.z = A2a1.z;
            }
            else{
                A2a.x = A2a2.x;
                A2a.y = A2a2.y;
                A2a.z = A2a2.z;
            }

            if(A2a.x != A2.x){
                double sl_r = (A2.z - A2a.z) / (A2.x - A2a.x);
                alpha2 = 180.0 * atan(sl_r) / PI;
            }
            else
                alpha2 = -90;
        }
    }
    else{
        Point B1 = Point();
        if(isAcceptable(A1, B11, D, -1)){
            B1.x = B11.x;
            B1.y = B11.y;
            B1.z = B11.z;
        }
        else{
            B1.x = B12.x;
            B1.y = B12.y;
            B1.z = B12.z;
        }

        if(A1.x != B1.x){
            double sl_n1 = (A1.z - B1.z) / (A1.x - B1.x);
            alpha1 = 180.0 * atan(sl_n1) / PI;
        }
        else
            alpha1 = -90;

        double beta1;
        if(D.x != B1.x){
            double sl_p = (D.z - B1.z) / (D.x - B1.x);
            beta1 = 180.0 * atan(sl_p) / PI;
        }
        else
            beta1 = -90;

        double xc = B1.x - F + L * cos(PI / 180.0 *(beta1 - PHI));
        double zc = B1.z + L * sin(PI / 180.0 *(beta1 - PHI));
        Point C = Point(xc, 0, zc);

        Circle K3 = Circle(A2, R);
        Circle K4 = Circle(C, N2);

        Point A2a1 = Point(), A2a2 = Point();
        K3.intersect(K4, A2a1, A2a2);

        if(std::isnan(A2a1.x) && std::isnan(A2a2.x))
            throw "Error in n2";
        else if(!std::isnan(A2a1.x) && std::isnan(A2a2.x)){
            Point A2a = Point(A2a1.x, A2a1.y, A2a1.z);;

            if(A2a.x != A2.x){
                double sl_r = (A2.z - A2a.z) / (A2.x - A2a.x);
                alpha2 = 180.0 * atan(sl_r) / PI;
            }
            else
                alpha2 = -90;
        }
        else{
            Point A2a = Point();
            if(isAcceptable(A2, A2a1, C, -1)){
                A2a.x = A2a1.x;
                A2a.y = A2a1.y;
                A2a.z = A2a1.z;
            }
            else{
                A2a.x = A2a2.x;
                A2a.y = A2a2.y;
                A2a.z = A2a2.z;
            }
            
            if(A2a.x != A2.x){
                double sl_r = (A2.z - A2a.z) / (A2.x - A2a.x);
                alpha2 = 180.0 * atan(sl_r) / PI;
            }
            else
                alpha2 = -90;
        }
    }
}

Point getD(double alpha1, double alpha2)
{
    Point B1 = Point(A1.x + N1 * cos(PI / 180 * alpha1), 0, A1.z + N1 * sin(PI / 180 * alpha1));
    Point B1a = Point(B1.x - F, 0, B1.z);

    Point A2a = Point(A2.x + R * cos(PI / 180 * alpha2), 0, A2.z + R * sin(PI / 180 * alpha2));

    double z = A2a.dist(B1a);
    double y = A2a.dist(B1);

    double mid = (z*z + L*L - N2*N2) / (2 * z * L);
    double zeta = acos(mid);
    double theta = acos((z*z + F*F - y*y) / (2 * z * F));
    double xda = B1a.x + P * cos(zeta + theta + PHI);
    double zda = B1a.z + P * sin(zeta + theta + PHI);
    Point Da = Point(xda, 0, zda);

    Point D = Point(Da.x + F, 0, Da.z);

    double al1 = alpha1, al2 = alpha2;
    if(alpha1 < -90)
        al1 = alpha1 + 180;
    if(alpha2 < -90)
        al2 = alpha2 + 180;

    double chk1 = 0,chk2 = 0;
    getAlphas(D, chk1, chk2);
    if(fabs(chk1 - al1) > 0.1 || fabs(chk2 - al2) > 0.1)
        throw "Not consistent with inverse model";
    
    return D;
}