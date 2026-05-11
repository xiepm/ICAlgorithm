#pragma once
#ifndef MATRIXALGORITHM_H
#define MATRIXALGORITHM_H

#include <iostream>

#define M 3  //¾ØÕó¼ÆËãµÄ½×Êý
#define N (2*M)
const double pi=3.1415926;
using namespace std;

void multiplyMatrix(double *&A, double *&B, double *&C,int m,int n);
void transposeMatrix(double A[][M], double B[][M]);
void inverseMatrix(double A[][M], double B[][M]);

#endif