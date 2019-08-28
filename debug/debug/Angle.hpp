//
//  Angle.hpp
//  debug
//
//  Created by fishming on 2019/8/26.
//  Copyright © 2019 fishming. All rights reserved.
//

#ifndef Angle_hpp
#define Angle_hpp
#include "Servo.hpp"
#include <stdio.h>
class Angle
{
public:
    Angle(int a_m, int a_n) : angMemory(a_m), angNow(a_n) {}
    Angle();
    void changeAngle(int ang);
    void angleReach(Servo& s);
    int angNow;
    
private:
    int angMemory;
};
#endif /* Angle_hpp */
