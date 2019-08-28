//
//  Matrix.cpp
//  debug
//
//  Created by fishming on 2019/8/26.
//  Copyright © 2019 fishming. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include "Matrix.h"

Matrix operator * (Matrix& m1, Matrix& m2)   //矩阵x乘法
{
    Matrix mTmp;
    mTmp.mArray.clear();
    try
    {
        if(m1.col != m2.row)
            throw std::invalid_argument("input error!");
    }
    catch(std::invalid_argument &ia)
    {
        std::cerr << ia.what() << std::endl;
        return mTmp;
    }
    for(int i = 0; i < m1.row;i++)   //重建mTmp
    {
        std::vector<double> colum;
        for(int j = 0; j < m2.col;j++)
        {
            colum.push_back(0);
        }
        mTmp.mArray.push_back(colum);
    }
    for(int i = 0;i < m1.row;i++)
    {
        for(int j = 0;j < m2.col;j++)
        {
            for(int tmp = 0;tmp < m2.row;tmp++)
            {
                mTmp.mArray[i][j] += m1.mArray[i][tmp] * m2.mArray[tmp][j];
            }
        }
    }
    return mTmp;
}

std::istream & operator >> (std::istream & input, Matrix &m)
{
    std::cout<<"请输入矩阵数据a行、b列"<<std::endl;
    std::cout<<"m: ";
    std::cin>>m.row;
    std::cout<<"n: ";
    std::cin>>m.col;
    m.mArray.clear();
    for(int i = 0; i < m.row;i++)
    {
        std::vector<double> colum;
        for(int j = 0; j < m.col;j++)
        {
            double d = 0;
            std::cin >> d;
            colum.push_back(d);
        }
        m.mArray.push_back(colum);
    }
    return input;
}

std::ostream & operator << (std::ostream & output, Matrix &m)
{
    std::vector<std::vector<double>>::const_iterator iter;
    for(iter = m.mArray.begin();iter < m.mArray.end();iter++)
    {
        for(std::vector<double>::const_iterator it = (*iter).begin();it < (*iter).end();it++)
        {
            std::cout << std::setw(8) << *it;
        }
        std::cout<<"\n"<<std::endl;
    }
    return output;
}

Matrix::Matrix()    //默认4x4
{
    mArray.clear();
    row = 4;col = 4;
    for(int i = 0;i < 4; i++)
    {
        std::vector<double> colum;
        for(int j = 0;j < 4;j++)
        {
            colum.push_back(0);
        }
        mArray.push_back(colum);
    }
}

void Matrix::input(double (*a)[4])
{
    row = 4;
    col = 4;
    mArray.clear();
    for(int i = 0;i < row; i++)
    {
        std::vector<double> colum;
        for(int j = 0;j < col;j++)
        {
            colum.push_back(a[i][j]);
        }
        mArray.push_back(colum);
    }
}

void Matrix::wrRowCol(int r, int c)
{
    row = r;
    col = c;
}
