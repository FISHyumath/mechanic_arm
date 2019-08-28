//
//  Servo.hpp
//  debug
//
//  Created by fishming on 2019/8/26.
//  Copyright © 2019 fishming. All rights reserved.
//

#ifndef Servo_hpp
#define Servo_hpp

#include <stdio.h>
class Servo
{
public:
    Servo();
    void attach(int, int, int);
    void write(int);
private:
    int PWM_min;
    int PWM_max;
    int PWM_now;
    int PIN;
    static int PWM_0;
    static int PWM_1;
};
#endif /* Servo_hpp */
