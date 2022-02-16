#include "non_parallel_util.h"
#include <iostream>
#include <stdio.h>
#include <time.h>

int main()
{
    Point D = Point(-1.32489278167, 0, -12.9563484818);
    double alpha1,alpha2;
    try{
        clock_t t;
        t = clock();
        getAlphas(D,alpha1,alpha2);
        std::cout<<"a1:"<<alpha1<<", a2:"<<alpha2<<"\n";
        t = clock() - t;
        double time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds
    
        std::cout<<"Time:"<<time_taken<<"\n";
    }
    catch(const char* exp){
        std::cout<<"An error occurred:"<<exp<<"\n";
    }
    
    return 0;
}