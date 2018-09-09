#include <stdio.h>
#include "matrix.h"

#define MIXDIM   100

void mat3mul(double s[3][3], double a[3][3], double b[3][3])
{
	s[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0];
	s[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1];
	s[0][2] = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2];
	s[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0];
	s[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1];
	s[1][2] = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2];
	s[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0];
	s[2][1] = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1];
	s[2][2] = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2];
}

void mat3tran(double s[3][3], double a[3][3])
{
	s[0][0] = a[0][0];  s[0][1] = a[1][0];	s[0][2] = a[2][0];
	s[1][0] = a[0][1];  s[1][1] = a[1][1];	s[1][2] = a[2][1];
	s[2][0] = a[0][2];  s[2][1] = a[1][2];	s[2][2] = a[2][2];
}

int mat3inv(double s[3][3], double a[3][3])
{
	double det;
	double *ps = (double*)s;
	int i;

	s[0][0] = a[1][1] * a[2][2] - a[2][1] * a[1][2];
	s[1][0] = a[2][0] * a[1][2] - a[1][0] * a[2][2];
	s[2][0] = a[1][0] * a[2][1] - a[2][0] * a[1][1];
	det = a[0][0]*s[0][0] + a[0][1]*s[1][0] + a[0][2]*s[2][0];
	if (det<1e-10 && det>-1e-10)
		return 0;
	else
	{
		s[0][1] = a[2][1] * a[0][2] - a[0][1] * a[2][2];
		s[1][1] = a[0][0] * a[2][2] - a[2][0] * a[0][2];
		s[2][1] = a[2][0] * a[0][1] - a[0][0] * a[2][1];
		s[0][2] = a[0][1] * a[1][2] - a[1][1] * a[0][2];
		s[1][2] = a[1][0] * a[0][2] - a[0][0] * a[1][2];
		s[2][2] = a[0][0] * a[1][1] - a[1][0] * a[0][1];
		for (i = 0; i < 9; i++)
			ps[i] /= det;
		return 1;
	}
}

double mat3det(double a[3][3])
{
	double s;

	s = a[0][0] * (a[1][1] * a[2][2] - a[2][1] * a[1][2]) -
		a[0][1] * (a[1][0] * a[2][2] - a[2][0] * a[1][2]) +
		a[0][2] * (a[1][0] * a[2][1] - a[2][0] * a[1][1]);

	return s;
}

void matrixcpy(double *dst, double *src, int m, int n)
{
	int i, l = m*n;

	for (i = 0; i < l; i++)
		dst[i] = src[i];
}

void matrixadd(double *s, double *a, double *b, int m, int n)
{
	int i, l = m*n;

	for (i = 0; i < l; i++)
		s[i] = a[i] + b[i];
}

void matrixsub(double *s, double *a, double *b, int m, int n)
{
	int i, l = m*n;

	for (i = 0; i < l; i++)
		s[i] = a[i] - b[i];
}

void matrixmul(double *s, double *a, double *b, int l, int m, int n)
{
	int i, j, k;
	double *pb;
	double tmp;

	for (i = 0; i < l; i++)
	{
		for (j = 0; j < n; j++)
		{
			pb = b + j;//move to next column.
			tmp = a[0] * (*pb);
			for (k = 1; k < m; k++)
			{
				pb += n;
				tmp += a[k] * (*pb);
			}
			*(s++) = tmp;
		}
		a += m;//move to next row.
	}
}

void matrixmulconst(double *s, double *a, double b, int m, int n)
{
	int i, l = m*n;

	for (i = 0; i < l; i++)
		s[i] = a[i] * b;
}

void matrixmuldiag(double *s, double *a, double *b, int m, int n)
{
	int i, j;

	for(i=0; i<m; i++)
	{
		for(j=0; j<n; j++)
			*(s++) = *(a++) * b[j];
	}
}

void matrixmultran(double *s, double *a, double *b, int l, int m, int n)
{
	int i, j, k;
	double *pb;
	double tmp;
	
	for (i = 0; i < l; i++)
	{
		for (j = 0; j < n; j++)
		{
			pb = b + j*m;//move to next row.
			tmp = a[0] * pb[0];
			for (k = 1; k < m; k++)
				tmp += a[k] * pb[k];
			*(s++) = tmp;
		}
		a += m;//move to next row.
	}
}

int matrixinv(double *s, double *a, int n)
{
	int column[MIXDIM];//store column order,the maximal dimension is MIXDIM.
	int i, j, k, index_x, index_y, tmpi;
	double *pa = a, *pb = a;
	double *ps = s, *pt = s;
	double max, tmpd;

	tmpi = n * n;
	for (i = 0; i < tmpi; i++)
	{
		s[i] = a[i];
		a[i] = 0;
	}//copy a to s,and set all elements of a to 0.
	tmpi = n + 1;
	for (i = 0; i < n; i++)
	{
		*pa = 1;//set a to identity matrix.
		pa += tmpi;
		column[i] = i;//initialize column order.
	}
	for (i = 0; i < n; i++)
	{
		ps = pt;
		pa = pb;
		max = -1;
		for (j = i; j < n; j++)
		{
			for (k = i; k < n; k++)
			{
				tmpd = ps[k];
				if (tmpd < 0)
					tmpd = -tmpd;
				if (tmpd > max)
				{
					max = tmpd;
					index_x = j;
					index_y = k;
				}
			}
			ps += n;
		}//find the maximal element.
		if (max < 1e-10)
			return 0;//can't evaluate the inverse matrix.
		if (index_x != i)
		{
			ps = s + index_x*n;//point to the row that needs to exchange.
			pa = a + index_x*n;
			for (j = 0; j < n; j++)
			{
				tmpd = ps[j];
				ps[j] = pt[j];
				pt[j] = tmpd;
				tmpd = pa[j];
				pa[j] = pb[j];
				pb[j] = tmpd;
			}
		}//exchange row.
		if (index_y != i)
		{
			ps = s + index_y;//point to the column that needs to exchange.
			pa = s + i;
			for (j = 0; j < n; j++)
			{
				tmpd = *ps;
				*ps = *pa;
				*pa = tmpd;
				ps += n;
				pa += n;
			}
			tmpi = column[i];
			column[i] = column[index_y];
			column[index_y] = tmpi;
		}//exchange column.
		ps = s;
		pa = a;
		for (j = 0; j < n; j++)
		{
			if (j != i)
			{
				tmpd = ps[i] / pt[i];
				for (k = 0; k <= i; k++)
					pa[k] -= pb[k] * tmpd;
				for (; k < n; k++)
				{
					ps[k] -= pt[k] * tmpd;
					pa[k] -= pb[k] * tmpd;
				}
			}
			ps += n;
			pa += n;
		}
		pt += n;//point the next row.
		pb += n;
	}//process along row
	ps = s;
	pa = a;
	for (i = 0; i < n; i++)
	{
		tmpd = ps[i];
		for (j = 0; j < n; j++)
			pa[j] /= tmpd;
		pa += n;
		ps += n;
	}//a divide s's diagonal element.
	for (i = 0; i < n; i++)
	{
		ps = s + n*column[i];
		pa = a + n*i;
		for (j = 0; j < n; j++)
			ps[j] = pa[j];
	}//exchange and copy row.

	return 1;
}

double matrixdet(double *a, int n)
{
	int i, j, k, index_x, index_y, sign = 1;
	double *pa, *pb = a, *ps;
	double max, tmpd;

	for (i = 0; i < n; i++)
	{
		pa = pb;
		max = -1;
		for (j = i; j < n; j++)
		{
			for (k = i; k < n; k++)
			{
				tmpd = pa[k];
				if (tmpd < 0)
					tmpd = -tmpd;
				if (tmpd > max)
				{
					max = tmpd;
					index_x = j;
					index_y = k;
				}
			}
			pa += n;
		}//find the maximal element.
		if (max < 1e-10)
			return 0.0;
		if (index_x != i)
		{
			pa = a + index_x*n;//point to the row that needs to exchange.
			for (j = 0; j < n; j++)
			{
				tmpd = pa[j];
				pa[j] = pb[j];
				pb[j] = tmpd;
			}
			sign *= -1;
		}//exchange row.
		if (index_y != i)
		{
			pa = a + index_y;//point to the column that needs to exchange.
			ps = a + i;
			for (j = 0; j < n; j++)
			{
				tmpd = *pa;
				*pa = *ps;
				*ps = tmpd;
				pa += n;
				ps += n;
			}
			sign *= -1;
		}//exchange column.
		pa = pb + n;
		for (j = i + 1; j < n; j++)
		{
			tmpd = pa[i] / pb[i];
			for (k = i + 1; k < n; k++)
				pa[k] -= pb[k] * tmpd;
			pa += n;
		}
		pb += n;//point the next row.
	}
	pa = a;
	tmpd = 1.0;
	for (i = 0; i < n; i++)
	{
		tmpd *= pa[i];
		pa += n;
	}
	
	return tmpd*sign;
}

void matrixtran(double *s, double *a, int m, int n)
{
	int i, j;
	double *ps;

	for (i = 0; i < m; i++)
	{
		ps = s + i;
		for (j = 0; j < n; j++)
		{
			*ps = *(a++);
			ps += m;
		}
	}
}

void matrixsym(double* a, int n)
{
	int i, j;

	for(i=1; i<n; i++)
	{
		for(j=0; j<i; j++)
		{
			*(a+n*i+j) = (*(a+n*i+j) + *(a+n*j+i)) / 2.0;
			*(a+n*j+i) = *(a+n*i+j);
		}
	}
}

void matrixzeros(double *a, int m, int n)
{
	int i, l = m*n;

	for (i = 0; i < l; i++)
		a[i] = 0;
}

void matrixeye(double *a, int n)
{
	int i, j = 0, l = n*n;

	for (i = 0; i < l; i++)
	{
		if (i == j)
		{
			a[i] = 1;
			j += n + 1;
		}
		else
			a[i] = 0;
	}
}

void matrixprint(double *a, int m, int n)
{
	int i,j;

	for (i = 0 ; i < m; i++)
	{
		for (j = 0; j < n; j++)
			printf("%17.10f", *(a++));
		printf("\n");
	}
	printf("-----------------------------------------------------------------\n");
}
