#pragma once
#include "config.h"

#define _USE_MATH_DEFINES

#define LASER_NUM 64
#define MAX_TIMES 100
#define FUCK_NUM 60
#define filter_length 129
#define MAX_POINTS 3000
#define CLOUD_NUM 256000
#define ANGLE_NUM 36000


using namespace std;

typedef struct
{
	int x;
	int y;
	int z;
	int c;
}spoint;

typedef struct
{
	int x;
	int y;
	int c;
}Savepoint;

typedef struct
{
	int x;
	int y;
	int z;
	unsigned short dist;
	unsigned short rot;
	unsigned char i;
	unsigned char c;
}Point_t;

typedef struct
{
	unsigned short dist;
	unsigned short rot;
	unsigned char i;
	unsigned char c;
}PointSave;

typedef struct
{
	int pts_count;
	Point_t pts[CLOUD_NUM];
}Points;

typedef struct
{
	int c;
	int p[ANGLE_NUM];
}layer_Points;

class HDLScale_t{
public:
	double x_min;
	double x_max;
	double y_min;
	double y_max;
	double x_scale;
	double y_scale;

	HDLScale_t();
};

class LineTypes
{
public:
	bool exist;
	double a,b,c,d,scale;
	int num;
	Savepoint pt[600];

	LineTypes();
	void set(double ta,double tb,double tc,double td,double _sa);
	void copy(Savepoint _pt[600],int count);
	LineTypes &operator = (const LineTypes &right);
};

float absd(float temp);
int tmin(int x,int y);
int tmax(int x,int y);

extern bool tpause;
