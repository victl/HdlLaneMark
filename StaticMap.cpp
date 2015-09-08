#include "StaticMap.h"
//ofstream testout("testout.txt");
//ofstream testoutt("testoutt.txt");
CvSize m_size = cvSize(1000,1000);
//vector<CvPoint> pl,pr;


StaticMap::StaticMap()
{
	rawImg = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_8U, 3);//img大小为500*500
	staticgrid = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_8U, 3);//img大小为500*500;
	dynamicgrid = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_8U, 3);//img大小为500*500;
	tmpimg = cvCreateImage(cvSize(1000, 1000), IPL_DEPTH_8U, 3);
	cvInitFont(&font, CV_FONT_VECTOR0, 0.75, 0.75, 1.0, 1, 3);
	gridNum = GRID_NUM;
	FrameNo = 0;
}

StaticMap::~StaticMap()
{

	if (tmpimg)
	{
		cvReleaseImage(&tmpimg);
		tmpimg = NULL;
	}
	if (rawImg)
	{
		cvReleaseImage(&rawImg);
		rawImg = NULL;
	}

	if (staticgrid)
	{
		cvReleaseImage(&staticgrid);
		staticgrid = NULL;
	}

	if (dynamicgrid)
	{
		cvReleaseImage(&dynamicgrid);
		dynamicgrid = NULL;
	}
	DynamicGridVec.clear();
	StaticGridVec.clear();
	LeftCarTrack.clear();
	RightCarTrack.clear();
	RightBoudary.clear();
	LeftBoundary.clear();
	ClearVector(DynamicGridVec);
	ClearVector(StaticGridVec);
	ClearVector(LeftCarTrack);
	ClearVector(RightCarTrack);
	ClearVector(RightBoudary);
	ClearVector(LeftBoundary);
}

void StaticMap::InitImage()
{
	cvCircle(staticgrid, cvPoint(staticgrid->width / 2, staticgrid->height / 2), 4, CV_RGB(254, 13, 34), -1);
	cvLine(tmpimg,cvPoint(tmpimg->width/2,0),cvPoint(tmpimg->width/2,tmpimg->height),CV_RGB(255,220,220),1);
	cvLine(tmpimg,cvPoint(0,tmpimg->height/2),cvPoint(tmpimg->width,tmpimg->height/2),CV_RGB(255,220,220),1);
	cvCircle(tmpimg, cvPoint(staticgrid->width / 2, staticgrid->height / 2), 4, CV_RGB(254, 213, 234), -1);
}
void StaticMap::Process()
{
	UpdateMap();
	//ShowStaticMap();
}

void StaticMap::Initialize()
{
	StaticGridVec.clear();
	PrioriStaticGridVec.clear();
	ClearVector(StaticGridVec);
	ClearVector(PrioriStaticGridVec);
	StaticGridVec = DynamicGridVec;
	PrioriStaticGridVec = DynamicGridVec;
}

void StaticMap::UpdateMap()
{
	long id;
	double S = 0;
	if (FrameNo == 0)
	{
		Initialize();
	}
	else
	{
		for (int i = 0; i<gridNum; i++)//row
		{
			for (int j = 0; j<gridNum; j++)//col
			{
				id = i*gridNum + j;
				long tid = (i - shift.y)*gridNum + j + shift.x;
				if (i>abs(shift.y>0?shift.y+3:shift.y) && j>abs(shift.x>0?shift.x+3:shift.x) && tid < gridNum*gridNum&& tid>=0)
					StaticGridVec[id] = PrioriStaticGridVec[tid];
				else// if(i<gridNum)//BUGGY 只处理了
					StaticGridVec[id].p = DynamicGridVec[id].p;
			}
		}

		for (size_t i = 0; i < gridNum; i++)//row
		{
			for (size_t j = 0; j < gridNum; j++)//col
			{

				id = i*gridNum + j;
				if(StaticGridVec[id].p>=0.3&&DynamicGridVec[id].p>0.3)
				{
					if (StaticGridVec[id].p < 0.5)
					{
						if (DynamicGridVec[id].p<0.5)
							if (StaticGridVec[id].p > DynamicGridVec[id].p)
								StaticGridVec[id].p = DynamicGridVec[id].p;

						if (DynamicGridVec[id].p >= 0.99)
						{
							S = (DynamicGridVec[id].p / (1 - DynamicGridVec[id].p))
								* (StaticGridVec[id].p / (1 - StaticGridVec[id].p));
							StaticGridVec[id].p = S / (1 + S);
							if (StaticGridVec[id].p > 0.99)
								StaticGridVec[id].p = 0.99;
							if (StaticGridVec[id].p <= 0.01)
								StaticGridVec[id].p = 0.01;
						}
					}
					if (StaticGridVec[id].p > 0.5)
					{
						if (DynamicGridVec[id].p > 0.5)
							if (StaticGridVec[id].p < DynamicGridVec[id].p)
								StaticGridVec[id].p = DynamicGridVec[id].p;

						if (DynamicGridVec[id].p <= 0.01)
						{
							S = (DynamicGridVec[id].p / (1 - DynamicGridVec[id].p))
								* (StaticGridVec[id].p / (1 - StaticGridVec[id].p));
							StaticGridVec[id].p = S / (1 + S);
							if (StaticGridVec[id].p > 0.99)
								StaticGridVec[id].p = 0.99;
							if (StaticGridVec[id].p <= 0.01)
								StaticGridVec[id].p = 0.01;
						}
					}

					if (StaticGridVec[id].p == 0.5)
						if (DynamicGridVec[id].p != 0.5)
							StaticGridVec[id].p = DynamicGridVec[id].p;
				}
				else//	if(StaticGridVec[id].p<0.3||StaticGridVec[id].p<0.3)
					StaticGridVec[id] .p= min(StaticGridVec[id].p,DynamicGridVec[id].p);
			}
		}
		PrioriStaticGridVec.clear();
		ClearVector(PrioriStaticGridVec);
		PrioriStaticGridVec = StaticGridVec;
	}
	FrameNo++;
}


void StaticMap::ShowStaticMap()
{
	int grid_size = rawImg->width / gridNum;
	cvZero(staticgrid);
	cvZero(tmpimg);
	//vector<double> lineParameters1, lineParameters2, lineP1, lineP2;
	CvScalar * left_clothoid = new CvScalar;
	CvScalar * right_clothoid = new CvScalar;
	CvScalar * leftClothoid = new CvScalar;
	CvScalar * rightClothoid = new CvScalar;
	double a, b, c, d;
	CvPoint ptemp;
//	int numForEstimate = 2;
	//CvPoint mp;
	//BoundaryGrids();
	//ObstacleDetect();
	for (size_t i = 0; i < gridNum; i++)// the gray map
	{
		for (size_t j = 0; j < gridNum; j++)
		{
			int row = i;
			int col = j;
			int c = P2Color(StaticGridVec[row*gridNum + col].p);
			if(c<=127)
			cvRectangle(staticgrid, cvPoint(col*grid_size, row*grid_size)
				, cvPoint((col + 1)*grid_size, (row + 1)*grid_size), CV_RGB(c, c, c), -1);
			else
				cvRectangle(staticgrid, cvPoint(col*grid_size, row*grid_size)
				, cvPoint((col + 1)*grid_size, (row + 1)*grid_size), CV_RGB(c, 0, 0), -1);
		}
	}


	InitImage();
	cvShowImage("staticgrid", staticgrid);
}

// void StaticMap::ShowDots(vector<CvPoint> pts,CvScalar co,IplImage * img,unsigned short r)
// {
// 	for (size_t i = 0; i < pts.size(); i++)
// 		cvCircle(img, pts[i], r, co,-1);
// }
// 
// int StaticMap::GetBelief(vector<CvPoint> pts,CvScalar * clothoid, IplImage * img)
// {
// 	double a,b,c,d;
// 	a = clothoid->val[0];
// 	b = clothoid->val[1];
// 	c = clothoid->val[2];
// 	d = clothoid->val[3];
// 	CvPoint ptemp;
// 	int result = 0;
// 	for(size_t i =0 ; i<pts.size();i++)
// 	{
// 		ptemp.y = pts[i].y;
// 		ptemp.x = a*ptemp.y*ptemp.y*ptemp.y + b*ptemp.y*ptemp.y + c*ptemp.y + d;
// 			cvCircle(img, cvPoint((int)ptemp.x, (int)ptemp.y), 0, CV_RGB(255, 25, 0), -1);
// 		if(abs(ptemp.x-pts[i].x)>10)
// 			cvCircle(img, cvPoint((int)pts[i].x, (int)pts[i].y), 3, CV_RGB(255, 0, 0), -1);
// 		result +=abs(ptemp.x-pts[i].x);
// 	} 
// 		return result/pts.size();
// }
// 
// vector<CvPoint> StaticMap::FitLine( CvScalar * clothoid,IplImage * img,int s,int e)
// {
// 	double a,b,c,d;
// 	a = clothoid->val[0];
// 	b = clothoid->val[1];
// 	c = clothoid->val[2];
// 	d = clothoid->val[3];
// 	CvPoint ptemp;
// 	vector<CvPoint> result;
// 	for (int i =s; i < e; i++)
// 	{
// 		ptemp.y = i;//i*6;
// 		ptemp.x = a*ptemp.y*ptemp.y*ptemp.y + b*ptemp.y*ptemp.y + c*ptemp.y + d;
// 		cvCircle(img, cvPoint((int)ptemp.x, (int)ptemp.y), 1, CV_RGB(25, 255, 25), -1);
// 		result.push_back(ptemp);
// 	}
// 	return result;
// }
// 
// vector<CvPoint> StaticMap::PreviewLine( CvScalar * clothoid,IplImage * img,int s)
// {
// 	double a,b,c,d;
// 	a = clothoid->val[0];
// 	b = clothoid->val[1];
// 	c = clothoid->val[2];
// 	d=m_size.width/2-(a*pow(m_size.width/2,3)+b*pow(m_size.width/2,2)+c*m_size.width/2);
// 	vector<CvPoint> result;
// 	CvPoint ptemp;
// 	for (int i =s; i < 500; i+=10)
// 	{
// 		ptemp.y = i;//i*6;
// 		ptemp.x = a*ptemp.y*ptemp.y*ptemp.y + b*ptemp.y*ptemp.y + c*ptemp.y + d;
// 		cvCircle(img, cvPoint((int)ptemp.x, (int)ptemp.y), 1, CV_RGB(25, 255, 25), -1);
// 		result.push_back(ptemp);
// 	}
// 	return result;
// }
// 
int StaticMap::P2Color(double p)
{
	int delta = 50;
	if (p<0.5&& p >= 0)
		return 255 * (1 - p);

	if (p>0.5&&p <= 1)
		return 127 - (p - 0.5) / 0.05*delta >= 0 ? 127 - (p - 0.5) / 0.05*delta : 0;

	return 127;
}


