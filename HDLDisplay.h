#pragma once
#include "config.h"
#include "config.h"
#include "DynamicMap.h"
#include "StaticMap.h"
#include "LocalMap.h"
#include "CurbScanner.h"
class HDLDisplay
{
public:
	HDLDisplay(void);
	~HDLDisplay(void);
	bool Initialize(const char* hdlfilepath);
	bool Process();

public:
	bool load_laser_info(const char* data_path);
	void show();
	void ishow();
	void comput(double &orix,double &oriy,double &orit);
	void showall();
	void ishowall();
	void findaccording(int q,int p,int minline);
	void moveleft();
	bool checkline(int p);
	void moveright();
	bool linefind();
	void filter();
	void check(int q,int p);
	void track(int p,int &minline);
	void find(IplImage* pod,CvPoint x0,CvPoint x1,int p);
	bool ransac();
	void ProcessDynamicMap();
	void ProcessStaticMap();
	void ProcessLocalMap();
	vector<CvPoint> Convert(vector<CvPoint> pts,double eulr);
	vector<CvPoint> Tranform( vector<CvPoint> pts);
	vector<CvPoint> ReTranform( vector<CvPoint> pts);
public:
	bool existleft,existright;
	//each time this->process() is invoked ( that is, a frame of HDL data was processed), sum is incremented by 1
	//m_svae_cloud_count is read from .hdl file header, indicating how many points were received in this frame.
	int nowcount,sum,m_save_cloud_count,csan;
	double sana,sanb,sanc,sand;
	double veix,veiy,veit,lastveiy,scax,scay,sca;
	double farlastx,farlasty,farlastt;	
	unsigned long m_timestamp;
	double eulr;
	FILE *m_hdldata,*m_veidata,*m_out;
	ifstream m_timestampdata;	
	CvSize m_size;
	IplImage *img,*allimg,*tim;
	IplImage *image,*mulImage;
	IplImage * xim;
	CvMat*   trans;
	CvPoint delta;
	Points m_cloud;
	HDLScale_t m_scale;
	//m_save_cloud is initialized to be a CLOUD_NUM (256000) array
	PointSave *m_save_cloud;
	size_t nCount;
	vector<Pose_t> m_vehicle_pos_t;
	LineTypes pline[4];
	spoint psan[30000];
	float gridsize ;
	float grid_size;
	CurbScanner * m_curb;
	//rasersort is used to sort laser beam according to their vertical angle
	int pointcount[MAX_TIMES],rasersort[LASER_NUM],layermark[LASER_NUM];
	double savex[MAX_TIMES],savey[MAX_TIMES],savet[MAX_TIMES];
	Savepoint savep[MAX_TIMES][MAX_POINTS];
	layer_Points m_layer_cloud[LASER_NUM];
	vector<CvPoint> m_l,m_r;
	DynamicMap * d_map;
	StaticMap * s_map;
	LocalMap * l_map;
	//set<int> Railgrids;
	vector<int> Railgridsvec;
	//vector<pair<int,int> > PointsInGrid;
	vector<unsigned short int > NumOfPointsInGrid;

private:
	float m_rot[LASER_NUM];  //for each laser: rot angle
	float m_vert[LASER_NUM];  //vertical angle correction
	float m_dist[LASER_NUM];  //distance system error
	float m_z_off[LASER_NUM]; //vertical offset
	float m_x_off[LASER_NUM]; //horizantal offset

	//S2 new features:
	float m_min_i[LASER_NUM]; //minIntensity
	float m_max_i[LASER_NUM]; //maxIntensity
	float m_distX[LASER_NUM]; //distCorrectionX
	float m_distY[LASER_NUM]; //distCorrectionY
	float m_f_d[LASER_NUM];  //focalDistance
	float m_f_s[LASER_NUM]; //focalSlope	


	float m_cos_rot[HDL_LASER_NUMBER];
	float m_sin_rot[HDL_LASER_NUMBER];
	float m_cos_vert[HDL_LASER_NUMBER];
	float m_sin_vert[HDL_LASER_NUMBER];
	float *m_cos_raw;
	float *m_sin_raw;
};


static float f[filter_length] = {  
  -0.000593547880000,  -0.000247614415645,   0.000008976984583,  -0.000058439301656,  -0.000447977647420,  -0.000894719875893,  -0.001050880642920,
  -0.000766550181149,  -0.000257760258803,   0.000008842021400,  -0.000319940189532,  -0.001138915606555,  -0.001878332373073,  -0.001923709972074,
  -0.001173344162611,  -0.000246309617908,  -0.000066845860312,  -0.001073835191384,  -0.002711662709667,  -0.003737496051440,  -0.003228887443726,
  -0.001497503203528,  -0.000042900510916,  -0.000402987785449,  -0.002752995018260,  -0.005495941720570,  -0.006400189726813,  -0.004516862993902,
  -0.001261463317032,   0.000468097862909,  -0.001336060020677,  -0.005834878906050,  -0.009638289493212,  -0.009445472207055,  -0.005019451042967,
   0.000125698904154,   0.001291767507565,  -0.003387358345762,  -0.010851444272282,  -0.015096805053840,  -0.012075967424608,  -0.003589660249813,
   0.003432437629167,   0.002306818242253,  -0.007438093200057,  -0.018687463057180,  -0.021859133535448,  -0.012995692051506,   0.001824169436406,
   0.010200077514124,   0.003296919961972,  -0.015716378538022,  -0.032260937071003,  -0.031021310700540,  -0.009296661810182,   0.017907553770884,
   0.026952123350483,   0.004018844549985,  -0.040285977859873,  -0.072563977302580,  -0.056214031980006,   0.022675859805643,   0.140977150760256,
   0.247239443265731,   0.289832176508932,   0.247239443265731,   0.140977150760256,   0.022675859805643,  -0.056214031980006,  -0.072563977302580,
  -0.040285977859873,   0.004018844549985,   0.026952123350483,   0.017907553770884,  -0.009296661810182,  -0.031021310700540,  -0.032260937071003,
  -0.015716378538022,   0.003296919961972,   0.010200077514124,   0.001824169436406,  -0.012995692051506,  -0.021859133535448,  -0.018687463057180,
  -0.007438093200057,   0.002306818242253,   0.003432437629167,  -0.003589660249813,  -0.012075967424608,  -0.015096805053840,  -0.010851444272282,
  -0.003387358345762,   0.001291767507565,   0.000125698904154,  -0.005019451042967,  -0.009445472207055,  -0.009638289493212,  -0.005834878906050,
  -0.001336060020677,   0.000468097862909,  -0.001261463317032,  -0.004516862993902,  -0.006400189726813,  -0.005495941720570,  -0.002752995018260,
  -0.000402987785449,  -0.000042900510916,  -0.001497503203528,  -0.003228887443726,  -0.003737496051440,  -0.002711662709667,  -0.001073835191384,
  -0.000066845860312,  -0.000246309617908,  -0.001173344162611,  -0.001923709972074,  -0.001878332373073,  -0.001138915606555,  -0.000319940189532,
   0.000008842021400,  -0.000257760258803,  -0.000766550181149,  -0.001050880642920,  -0.000894719875893,  -0.000447977647420,  -0.000058439301656,
   0.000008976984583,  -0.000247614415645,  -0.000593547880000};
