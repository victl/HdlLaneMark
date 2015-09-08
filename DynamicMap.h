#include "config.h"
class DynamicMap
{
public:
	DynamicMap();
	~DynamicMap();
	vector<Grid> MeasurementGrid;
	map<int,unsigned char> PointsInGrid;
	map<int,unsigned char> TrafficSignGrid;
	short gridNum;
	double grid_size;
	CvPoint delta;
	IplImage * gridmap;
	float eulr;
	//CvPoint delta;
	void Process();
	void BuildMap();
	void ProbabilityMap();
	void ShowDynamicMap();
	int P2Color(float p);
	void Clean();
	double Likelihood(double d);
	void FrontPoints(int angle,int axis,int MaxAngle,int MinAnlge,map<int,set<double> > &anglemap);

private:

};

