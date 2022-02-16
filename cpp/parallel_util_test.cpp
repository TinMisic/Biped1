#include "parallel_util.h"
#include<iostream>
//#include<cmath>

int main(){
    Point A = Point(0,0,2);
    Point B = Point(0,0,0);
    std::cout<<F<<std::endl;

    Linear l1 = Linear(B,2.0);
    std::cout<<"Linear l1(2)="<<l1.get_z(2)<<"\n";

    Circle K1 = Circle(A,1.0);
    Circle K2 = Circle(B,1.0);

    Point first = Point();
    Point second = Point();

    K1.intersect(K2,first,second);

    if(std::isnan(first.x))
        std::cout<<"No intersect\n";
    else if(std::isnan(second.x)){
        std::cout<<"Only one Point\n";
        std::cout<<first.x<<","<<first.y<<","<<first.z<<"\n";
    }
    else{
        std::cout<<"Two points of intersection\n";
        std::cout<<first.x<<","<<first.y<<","<<first.z<<"\n";
        std::cout<<second.x<<","<<second.y<<","<<second.z<<"\n";
    }

    Point D = Point(20, 0, 20);
    double alpha1,alpha2;
    try{
        getAlphas(D,alpha1,alpha2);
        std::cout<<"a1:"<<alpha1<<", a2:"<<alpha2<<"\n";
    }
    catch(const char* exp){
        std::cout<<"An error occurred:"<<exp<<"\n";
    }
    return 0;
}