//
//  Angle.cpp
//  debug
//
//  Created by fishming on 2019/8/26.
//  Copyright © 2019 fishming. All rights reserved.
//

#include "Angle.hpp"

void Angle::angleReach(Servo& s) //快速跳转脱机角度可能发生堵转，这个舵机需要缓慢调整才行
{
    int angleTmp = angMemory;
    bool turningUp = false;
    if (angNow > angMemory)
    {
        turningUp = true; //缓慢调整上加角度
    }
    else
    {
        turningUp = false; //缓慢下降角度
    }
    if (turningUp)
    {
        while (angleTmp < angNow)
        {
            angleTmp++;
            s.write(angleTmp);
        }
    }
    else
    {
        while (angleTmp > angNow)
        {
            angleTmp--;
            s.write(angleTmp);
        }
    }
}
Angle::Angle()
{
    
}
void Angle::changeAngle(int ang)
{
    angMemory = angNow;
    angNow = ang;
}
