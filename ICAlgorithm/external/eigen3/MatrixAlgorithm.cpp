#pragma once

#include "stdafx.h"
#include "MatrixAlgorithm.h"

void multiplyMatrix(double *&A, double *&B, double *&C,int m,int n)
{
	
}

void inverseMatrix(double A[][M], double result[][M])
{
	int i,j,k;
	double B[M][N];
	for(i=0;i<M;i++)
	{
		for(j=0;j<M;j++)
		{
			B[i][j]=A[i][j];
		}
	}
    //扩展矩阵
	for(i=0;i<M;i++)
	{
		for(j=M;j<N;j++)
		{
			if(i==(j-M))
			{
				B[i][j]=1;
			}
			else
			{
				B[i][j]=0;
			}
		}
	}
	//求逆模块
	double temp;
	for(i=0;i<M;i++)
	{
		if(B[i][i]==0)
		{
			for(k=i;k<M;k++)
			{
				if(B[k][k]!=0)
				{
					for(int j=0;j<N;j++)
					{						
						temp=B[i][j];
						B[i][j]=B[k][j];
						B[k][j]=temp;
					}
					break;
				}
			}
			if(k==M)
			{
				cout<<"该矩阵不可逆！"<<endl;
			}
		}
		for(j=N-1;j>=i;j--)
		{
			B[i][j]/=B[i][i];
		}
		for(k=0;k<M;k++)
		{
			if(k!=i)
			{
				temp=B[k][i];
				for(j=0;j<N;j++)
				{
					B[k][j]-=temp*B[i][j];
				}
			}
		}
	}
	//导出结果
	for(i=0;i<M;i++)
	{
		for(j=M;j<N;j++)
		{
			result[i][j-M]=B[i][j];
		}
	}
}