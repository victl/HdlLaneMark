#include "config.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#define _USE_MATH_DEFINES
#include "math.h"



class LocalMap{
public:
	LocalMap();
	~LocalMap();
	void Initialize();
	void Process();
	void ShowLocalMap(size_t frame);
	void ShowLocalMap();
	void UpdateMap();
	int P2Color(double p);
	void SaveMap();
	void SaveMapWhole();
	void BoundaryGrids();
	void SaveFrame(size_t fame);
	CvScalar * fun(vector<CvPoint> pts);//char symbol,CvScalar line1,CvScalar line2,CvScalar * clothoid)
	int GetBelief(vector<CvPoint> pts,CvScalar * clothoid, IplImage * img);
	vector<CvPoint> FitLine( CvScalar * clothoid,IplImage * img,int s,int e,CvScalar co);
	vector<CvPoint> PreviewLine( CvScalar * clothoid,IplImage * img,int s,CvScalar co,unsigned char r);
	vector<CvPoint> Convert(vector<CvPoint> pts,double eulr);
	vector<CvPoint> Tranform( vector<CvPoint> pts);
	vector<CvPoint> ReTranform( vector<CvPoint> pts);
	vector<CvPoint> Bresenham(CvPoint p, CvPoint p1,float threshold);
	void ShowDots(vector<CvPoint> pts,CvScalar co,IplImage * img,unsigned short r);
	void puttext(CvPoint pt, int num,IplImage * img);//, char * text);
	void RailScanner( );//vector<CvPoint> Rpts,vector<CvPoint> Lpts, vector<CvPoint> Rpts0,vector<CvPoint>Lpts0);
	void CurbScanner();
	void Record();
public:
	size_t gridNum;
	//size_t FrameNO;
	size_t  TravelledDistance;
	IplImage * staticgrid;
	IplImage * localgrid;
	IplImage* tmpimg;
	//pointT * local_vec;
	CvPoint shift;
	pointT carpos;
	map<pointT,Grid> LocalGridMap;//,LocalGridMap0;
	map<pointT,unsigned char> gmap;
	map<pointT,unsigned short int > curbgrids;
	vector<Grid> GridVec;//Static Grid
	vector<CvPoint> leftRail,rightRail;
	CvSize m_size; 
	CvFont font;
	size_t FrameNO;
	double eulr;
	vector<CvPoint> m_l;
	vector<CvPoint> m_r;
	vector<unsigned char> layers;// [GRID_NUM][GRID_NUM];
	//set<pointT> right_curb,left_curb;
	map<pointT,int> right_curb,left_curb;
	vector<CvPoint> RC,LC;
//	map<pointT,float> carposmap;
	vector<carPose> carposvec;
	set<pointT> Railgrids;
	set<pointT> Trunk;
	vector<int> RailGridsvec;
	vector<unsigned short int > NumOfPointsInGrid;
	//pair<int,int> mincarpos;
};