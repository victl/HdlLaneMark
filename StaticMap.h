#include "config.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#define _USE_MATH_DEFINES
#include "math.h"

//#include "Grid.h"
//#include "LUXDisplay.h"

class StaticMap
{
public:
	StaticMap(void);
	~StaticMap(void);
	void Initialize();
	vector<CvPoint> FitLine( CvScalar * clothoid, IplImage * img,int s,int e);
	vector<CvPoint> PreviewLine( CvScalar * clothoid, IplImage * img,int s);

	void ShowDots(vector<CvPoint> pts,CvScalar co,IplImage * img,unsigned short r);
	void ShowStaticMap();
	int GetBelief(vector<CvPoint> pts,CvScalar * clothoid, IplImage * img);
	void Process();
	void Clean();
	void UpdateMap();
	void BoundaryGrids();
	void ObstacleDetect();
	void InitImage();
	CvScalar * fun(vector<CvPoint> pts);//char symbol,CvScalar line1,CvScalar line2,CvScalar * clothoid)
	int P2Color(double p);
	vector<CvPoint> Bresenham(CvPoint p, CvPoint p1,float threshold);
	void puttext(CvPoint pt, int num,IplImage * img);//, char * text)

public:
	vector<Grid> DynamicGridVec;//Current Dynamic Grid
	vector<Grid> StaticGridVec;//Static Grid
	vector<Grid> PrioriStaticGridVec;//Prior Grid
	vector<Grid> LeftCarTrack,RightCarTrack;// record the car track!
	vector<Grid> LeftBoundary;
	vector<Grid> RightBoudary;
	map<long,int> PointsInGrid;
	size_t gridNum;
	IplImage * rawImg;
	IplImage * staticgrid;
	IplImage * tmpimg;
	IplImage * dynamicgrid;
	int FrameNo;
	CvPoint  shift;
	CvPoint  carpos;
	CvFont font;
	map<int,set<double> > anglemap;
	map<long,set<unsigned char> > ChannelInGrid;
	vector<unsigned short int> lines;
	vector<unsigned short int> lines0;
	double al;
	double eulr;
	pair<int, int> edges;
	int belief,Lbelief,Rbelief;
	vector<CvPoint> EdgePts;
	vector<CvPoint> PrePts;
	vector<CvPoint> leftRail,rightRail;
	pair<float, float> l;
//	CvPoint adjust;
};
