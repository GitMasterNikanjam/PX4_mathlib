/*
mkdir build -p && g++ -o build/exp1 exp1.cpp 

*/
#include "iostream"
#include "../mathlib.h"

int main(void)
{
    float v = math::constrain(10.2, 0.1, 9.123);
    std::cout << "v = " << v << std::endl;
    return 0;
}