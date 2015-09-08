#include "config.h"
#include "defines.h"


float absd(float temp)
{
	if (temp<0) return -temp;
	return temp;
}

int tmin(int x,int y)
{
	return x>y?y:x;
}

int tmax(int x,int y)
{
	return x>y?x:y;
}

HDLScale_t::HDLScale_t()
{
    x_max = 5;
    x_min = -5;
    y_max = 10;
	y_min = 0;
	x_scale = 500 / (x_max - x_min);
	y_scale = 500 / (y_max - y_min);
}

LineTypes::LineTypes()
{
	exist=false;
}
	
void LineTypes::set(double ta,double tb,double tc,double td,double _sa)
{
	a=ta;b=tb;c=tc;d=td;scale=_sa;
}
	
void LineTypes::copy(Savepoint _pt[600],int count)
{
	num=0;int tp;
	for (int i=0;i<count;i++) 
	{
		tp=_pt[i].y;
		if (abs(a*tp*tp*tp+b*tp*tp+c*tp+d-_pt[i].x)<=5)
		{
			pt[num].x=_pt[i].x;pt[num].y=_pt[i].y;num++;
		}
	}
	if (num>0) exist=true;
}
	
LineTypes& LineTypes::operator = (const LineTypes &right)
{
	exist=right.exist;num=right.num;
	set(right.a,right.b,right.c,right.d,right.scale);
	for (int i=0;i<num;i++)
		{pt[i].x=right.pt[i].x;pt[i].y=right.pt[i].y;}
	return *this;
}
