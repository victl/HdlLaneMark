#pragma once

#define HDL_LASER_NUMBER       64
#define HDL_MAX_POINT_LASER    4000
#define HDL_MAX_POINT_NUMBER HDL_MAX_POINT_LASER * HDL_LASER_NUMBER

typedef struct LPoint
{
	int x;
	int y;
	int z;
	unsigned short dist;
	unsigned short rot;  // 旋转角
	unsigned char i;     // 强度信息
	unsigned char c;     // 扫描线序号
}LPoint_t;

typedef struct Pose
{
	double x;
	double y;
	double eulr;
}Pose_t;