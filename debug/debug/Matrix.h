//
//  Matrix.h
//  debug
//
//  Created by fishming on 2019/8/26.
//  Copyright © 2019 fishming. All rights reserved.
//

#ifndef Matrix_h
#define Matrix_h
#include <iostream>
#include <vector>

class Matrix    //仅仅为了调试方便，不会在arduino中浪费内存
{
public:
    Matrix();
    friend Matrix operator * (Matrix& m1,Matrix& m2);
    friend std::istream & operator >> (std::istream &, Matrix &m);
    friend std::ostream & operator << (std::ostream &, Matrix &m );
    void wrRowCol(int,int);
    void input(double a[][4]);  //这个目前仅支持4x4
    std::vector<std::vector<double>> mArray;    //长宽动态
    int row,col;
};


#endif /* Matrix_h */
