#include "config.h"
#include "DynamicMap.h"
DynamicMap::DynamicMap()
{
	gridNum = GRID_NUM;
	grid_size = 1000 / gridNum;
	gridmap = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_8U, 3);
	PointsInGrid.clear();
	//ClearVector(PointsInGrid);
//	PointsInGrid.resize(gridNum*gridNum, make_pair(0, 0));
}

DynamicMap::~DynamicMap()
{
	PointsInGrid.clear();
//	ClearVector(PointsInGrid);
}

void DynamicMap::Process()
{
	ProbabilityMap();
	//ShowDynamicMap();
	//cout<<"point s in  girds count:\t"<<PointsInGrid.size()<<endl;//"\t"<<nx<<"\t"<<ny<<endl;

//	Clean();
}

void DynamicMap::ShowDynamicMap()
{
	cvZero(gridmap);
	for (size_t i = 0; i < gridNum; i++)
	{
		for (size_t j = 0; j < gridNum; j++)
		{
			int row = i;
			int col = j;
			int c = P2Color(MeasurementGrid[row*gridNum + col].p);
				cvRectangle(gridmap, cvPoint(col*grid_size, row*grid_size)
				, cvPoint((col + 1)*grid_size, (row + 1)*grid_size), CV_RGB(c, c, c), -1);
		}
	}
	cvShowImage("gridmap", gridmap);
}

void DynamicMap::ProbabilityMap()
{
	Grid g;
	g.p = 0.5;
	float r = 0.25;
	MeasurementGrid.clear();
	ClearVector(MeasurementGrid);
	MeasurementGrid.resize(GRID_NUM*GRID_NUM,g);
	int n = 0;
	for(map<int,unsigned char>::iterator itr= PointsInGrid.begin();itr!=PointsInGrid.end();itr++)
		{
			n= itr->second;// (PointsInGrid[id]);//.first - PointsInGrid[id].second);//:0;
			g.p = 0.5 + r*n <= 1 ? 0.5 + r*n : 1;
			if(itr->first>=0&&itr->first<GRID_NUM*GRID_NUM)
				MeasurementGrid[itr->first] = g;
		}
	for (map<int, unsigned char>::iterator itr = TrafficSignGrid.begin(); itr != TrafficSignGrid.end(); itr++)
	{
		n = itr->second;// (PointsInGrid[id]);//.first - PointsInGrid[id].second);//:0;
		g.p = 0.5 - r*n >= 0 ? 0.5 - r*n : 0;
		if (itr->first >= 0 && itr->first<GRID_NUM*GRID_NUM)
			MeasurementGrid[itr->first] = g;
	}
}



void DynamicMap::Clean()
{
	PointsInGrid.clear();
	//ClearVector(PointsInGrid);
	//PointsInGrid.resize(gridNum*gridNum, make_pair(0, 0));
}

int DynamicMap::P2Color(float p)
{
	int delta = 50;
	if (p<0.5&& p >= 0)
		return 255 * (1 - p);

	if (p>0.5&&p <= 1)
		return 127 - (p - 0.5) / 0.05*delta >= 0 ? 127 - (p - 0.5) / 0.05*delta : 0;

	return 127;
}

double DynamicMap::Likelihood(double d)
{
	double Dmax = 1000;
	double Dmin = 2;
	if (d >= Dmax)
		return 0.5;

	if (d <= Dmin&&d >= 0)
		return  0.4*Dmin / Dmax;

	if (d > Dmin && d < Dmax)
		return		0.4*d / Dmax;

	return 0.5;
}

void DynamicMap::FrontPoints(int angle, int axis, int MaxAngle, int MinAngle, map<int, set<double> > &anglemap)
{
	double ratio = 0.2;
	int row, col;
	double R;
	int height = gridmap->height;
	int width = gridmap->width;
	int grid_size = width / gridNum;

	for (size_t i = 0; i < gridNum; i++)
	{
		row = 0;
		col = i;
		R = sqrt(pow((col + 0.5)*grid_size - width / 2, 2) + pow(height / 2 - (row + 0.5)*grid_size, 2));
		double a = (acos(((col + 0.5)*grid_size - width / 2) / R));
		int line = int(a / M_PI*angle);
		//	if(abs(int(a/M_PI*angle)-axis)<0.25*angle)
		if (row <= gridNum / 2)
		{
			if (abs(line - axis) < ratio*angle)
				anglemap[line].insert(R);
		}
		else
		{
			if (abs(2 * angle - 1 - line - axis) < ratio*angle)
				anglemap[2 * angle - 1 - line].insert(R);
		}

		row = gridNum - 1;
		col = i;
		R = sqrt(pow((col + 0.5)*grid_size - width / 2, 2) + pow(height / 2 - (row + 0.5)*grid_size, 2));
		a = (acos(((col + 0.5)*grid_size - width / 2) / R));
		line = int(a / M_PI*angle);
		//	if(abs(line-axis)<0.25*angle)
		if (row <= gridNum / 2)
		{
			if (abs(line - axis) < ratio*angle)
				anglemap[line].insert(R);
		}
		else
		{
			if (abs(2 * angle - 1 - line - axis) < ratio*angle)
				anglemap[2 * angle - 1 - line].insert(R);
		}

		row = i;
		col = 0;
		R = sqrt(pow((col + 0.5)*grid_size - width / 2, 2) + pow(height / 2 - (row + 0.5)*grid_size, 2));
		a = (acos(((col + 0.5)*grid_size - width / 2) / R));
		line = int(a / M_PI*angle);
		//	if(abs(line-axis)<0.25*angle)
		if (row <= gridNum / 2)
		{
			if (abs(line - axis) < ratio*angle)
				anglemap[line].insert(R);
		}
		else
		{
			if (abs(2 * angle - 1 - line - axis) < ratio*angle)
				anglemap[2 * angle - 1 - line].insert(R);
		}

		row = i;
		col = gridNum - 1;
		R = sqrt(pow((col + 0.5)*grid_size - width / 2, 2) + pow(height / 2 - (row + 0.5)*grid_size, 2));
		a = (acos(((col + 0.5)*grid_size - width / 2) / R));
		line = int(a / M_PI*angle);
		//		if(abs(line-axis)<0.25*angle)
		if (MinAngle < MaxAngle)
		{
			if (row <= gridNum / 2)
			{
				if (abs(line - axis) < ratio*angle)
					anglemap[line].insert(R);
			}
			else
			{
				if (abs(2 * angle - 1 - line - axis) < ratio*angle)
					anglemap[2 * angle - 1 - line].insert(R);
			}
		}
		else
		{
			if (row <= gridNum / 2)
			{
				if (line<MaxAngle)
					anglemap[line].insert(R);
			}
			else
			{
				if (2 * angle - 1 - line>MinAngle)
					anglemap[2 * angle - 1 - line].insert(R);
			}
		}
	}
}
