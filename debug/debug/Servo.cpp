//
//  Servo.cpp
//  debug
//
//  Created by fishming on 2019/8/26.
//  Copyright © 2019 fishming. All rights reserved.
//

#include "Servo.hpp"
int Servo::PWM_0 = 500;
int Servo::PWM_1 = 2500;
Servo::Servo()
{
    
}

void Servo::attach(int pin, int min, int max)
{
    pin = PIN;
    PWM_min = min;
    PWM_max = max;
}
void Servo::write(int angle)
{
    PWM_now = angle * (PWM_1 - PWM_0) / 180 + 500;
}
