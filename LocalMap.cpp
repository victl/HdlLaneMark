#include "LocalMap.h"
#include "math.h"
#include <string>
//ofstream  ygr("ygr.txt");
//extern CvSize m_size;// = cvSize(1000,1000);
//ofstream record("recorder.txt");
ofstream carposeout;//("carpose.txt");

vector<CvPoint> LocalMap::Convert(vector<CvPoint> pts,double eulr)
{
	vector<CvPoint> result;
	CvPoint tp;
	for(size_t i = 0;i<pts.size();i++)
	{
		tp.x = pts[i].x *cos(eulr) + pts[i].y*sin(eulr);
		tp.y = pts[i].y* cos(eulr) - pts[i].x *sin(eulr);
		result.push_back(tp);
	}
	return result;
}
//
vector<CvPoint> LocalMap::Tranform( vector<CvPoint> pts)
{
	vector<CvPoint> result;
	CvPoint tp;
	for(size_t i = 0;i<pts.size();i++)
	{
		tp.x = pts[i].x -m_size.width/2;
		tp.y = m_size.height/2 - pts[i].y;
		result.push_back(tp);
	}
	return result;
}
//
vector<CvPoint> LocalMap::ReTranform( vector<CvPoint> pts)
{
	vector<CvPoint> result;
	CvPoint tp;
	for(size_t i = 0;i<pts.size();i++)
	{
		tp.x = pts[i].x+m_size.width/2;
		tp.y = m_size.height/2 - pts[i].y;
		result.push_back(tp);
	}
	return result;
}


LocalMap::LocalMap()
{
	staticgrid = cvCreateImage(cvSize(1000,1000),IPL_DEPTH_8U,3);//img��СΪ500*500;
	localgrid = cvCreateImage(cvSize(1000,1000),IPL_DEPTH_8U,3);//img��СΪ500*500;
	 tmpimg = cvCreateImage(cvSize(1000,1000),IPL_DEPTH_8U,3);//img��СΪ500*500;
	 NumOfPointsInGrid.resize(GRID_NUM*GRID_NUM,0);
	gridNum = GRID_NUM;
	TravelledDistance = 0;
	m_size = cvSize(1000,1000);
	FrameNO = 0;
}

LocalMap::~LocalMap()
{
	if(staticgrid)
	{
		cvReleaseImage(&staticgrid);
		staticgrid = NULL;
	}
	if(localgrid)
	{
		cvReleaseImage(&localgrid);
		localgrid = NULL;
	}
	if(tmpimg)
	{
		cvReleaseImage(&tmpimg);
		tmpimg = NULL;
	}

	LocalGridMap.clear();
	GridVec.clear();
	ClearVector(GridVec);
}

void LocalMap::Initialize()
{
	;
}


void LocalMap::puttext(CvPoint pt, int num,IplImage * img)//, char * text)
{
	std::string numstr = std::to_string(num);
	cvPutText(img, numstr.c_str(), pt, &font, CV_RGB(0, 200, 255));
}



void LocalMap::ShowLocalMap(size_t frame)
{
	int grid_size = localgrid->width/gridNum;
	cvZero(tmpimg);
	cvZero(localgrid);
	pointT localpoint;
	CvScalar * left_clothoid = new CvScalar;
	CvScalar * right_clothoid = new CvScalar;
	CvScalar * leftClothoid = new CvScalar;
	CvScalar * rightClothoid = new CvScalar;
	//double a, b, c, d;
	CvPoint ptemp;
	int c,row,col;
	std::string framestr = std::to_string(frame);
	char  filename[100];//= "mappp.gm";
	IplImage * img = cvCreateImage(cvSize(GRID_NUM, GRID_NUM), 8, 3);

	for(size_t  i = 0;i<gridNum;i++)// the gray map
		for(size_t j=0;j<gridNum;j++)
		{
			row =i;
			col = j;
			localpoint.x = carpos.x -(gridNum)/2+col;
			localpoint.y = carpos.y +(gridNum)/2-row;
			c =127;
			if(gmap.count(localpoint))
				 c = gmap[localpoint];//P2Color(LocalGridMap[localpoint].p);
			if(c==4)
			{
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
					,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(0,255,0),-1);
				cvSet2D(img,i,j,CV_RGB(0,255,0));
			}
			else if(c==5)
			{
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
				,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(255,128,128),-1);
				cvSet2D(img,i,j,CV_RGB(255,128,128));
			}
			else if(c==6)
			{
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
				,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(0,0,255),-1);
				cvSet2D(img,i,j,CV_RGB(0,0,255));
			}
			else if(c==255)
			{
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
					,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(c,0,0),-1);
				cvSet2D(img,i,j,CV_RGB(c,0,0));
			}
			else if(c==0)
			{
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
					,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(c,c,c),-1);
				cvSet2D(img,i,j,CV_RGB(0,0,0));
			}
			else
			{
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
					,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(c,c,c),-1);
				cvSet2D(img,i,j,CV_RGB(c,c,c));
			}
		}

	sprintf(filename, "map_%s.png", framestr.c_str());//,cm);//,cm);
	cvSaveImage(filename, img);
	cvLine(localgrid, cvPoint(localgrid->width / 2 + (localgrid->height / 2)*tan(eulr), 0),cvPoint(localgrid->height / 2 - (localgrid->height / 2)*tan(eulr), localgrid->height),CV_RGB(0, 55, 255));
	cvLine(localgrid, cvPoint(localgrid->width / 2 - (localgrid->height / 2)/tan(eulr), 0),cvPoint(localgrid->height / 2 + (localgrid->height / 2)/tan(eulr), localgrid->height),	CV_RGB(0, 55, 255));
	cvCircle(localgrid, cvPoint(localgrid->width / 2, localgrid->height / 2), 4, CV_RGB(254, 13, 34), -1);
	cvShowImage("local grid0",localgrid);
	//cvReleaseImage(&img);
}


void LocalMap::ShowLocalMap()
{
	int grid_size = localgrid->width/gridNum;
	cvZero(tmpimg);
	cvZero(localgrid);
	pointT localpoint;
	CvScalar * left_clothoid = new CvScalar;
	CvScalar * right_clothoid = new CvScalar;
	CvScalar * leftClothoid = new CvScalar;
	CvScalar * rightClothoid = new CvScalar;
	//double a, b, c, d;
	CvPoint ptemp;
	int c,row,col;
	char  cn[10];
	char  filename[100];//= "mappp.gm";
	for(size_t  i = 0;i<gridNum;i++)// the gray map
		for(size_t j=0;j<gridNum;j++)
		{
			row =i;
			col = j;
			localpoint.x = carpos.x -(gridNum)/2+col;
			localpoint.y = carpos.y +(gridNum)/2-row;
			c =127;
			if(gmap.count(localpoint))
				c = gmap[localpoint];//P2Color(LocalGridMap[localpoint].p);
			if(c==4)
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
				,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(0,255,2),-1);
			else if(c==5)
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
				,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(240,128,128),-1);
			else if(c==6)
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
				,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(0,12,248),-1);
			else
				cvRectangle(localgrid,cvPoint(col*grid_size ,row*grid_size) 
				,cvPoint((col+1)*grid_size ,(row+1)*grid_size),CV_RGB(c,c,c),-1);
		}

		cvLine(localgrid, cvPoint(localgrid->width / 2 + (localgrid->height / 2)*tan(eulr), 0),cvPoint(localgrid->height / 2 - (localgrid->height / 2)*tan(eulr), localgrid->height),CV_RGB(0, 55, 255));
		cvLine(localgrid, cvPoint(localgrid->width / 2 - (localgrid->height / 2)/tan(eulr), 0),cvPoint(localgrid->height / 2 + (localgrid->height / 2)/tan(eulr), localgrid->height),	CV_RGB(0, 55, 255));
		cvCircle(localgrid, cvPoint(localgrid->width / 2, localgrid->height / 2), 4, CV_RGB(254, 13, 34), -1);
		cvShowImage("local grid0",localgrid);
		//cvReleaseImage(&img);
}


void LocalMap::SaveMapWhole()
{
	unsigned short int MAPHET, MAPWID;
	cout<<"Writing the map file..."<<endl;
	 ofstream mapout("map.gm",ios::binary);
	 size_t sizeofmap = gmap.size();
	 mapout.write((char*)&sizeofmap,sizeof(sizeofmap));
	 pointT pt;
	 pointT_ pt_;
	 unsigned char uc;

	 int minx,miny,maxx,maxy;
		 minx = gmap.begin()->first.x;//carpos.x;//car_pose_vec[0].x;
		 maxx = gmap.begin()->first.x;//carpos.x;// car_pose_vec[0].x;
		 miny = gmap.begin()->first.y;//carpos.y;//car_pose_vec[0].y;
		 maxy = gmap.begin()->first.y;//carpos.y;// car_pose_vec[0].y;
	 for (map<pointT, unsigned char>::iterator itr = gmap.begin(); itr != gmap.end(); itr++)//for (size_t i = 0; i < local_grid_points_vec.size(); i++)
	 {
		 if (minx >itr->first.x)
			 minx = itr->first.x;
		 if (miny> itr->first.y)
			 miny = itr->first.y;
		 if (maxx <itr->first.x)
			 maxx = itr->first.x;
		 if (maxy <itr->first.y)
			 maxy = itr->first.y;
	 }
	 mapout.write((char*)&minx, sizeof(minx));
	 mapout.write((char*)&miny, sizeof(miny));
	 mapout.write((char*)&maxx, sizeof(maxx));
	 mapout.write((char*)&maxy, sizeof(maxy));
	 MAPWID = abs(maxx - minx);
	 MAPHET = abs(maxy - miny);
	 for(map<pointT,unsigned char>::iterator itr= gmap.begin();itr!=gmap.end();itr++)//for (size_t i = 0; i < local_grid_points_vec.size(); i++)
	 {
		 pt_.x = itr->first.x - minx;// - mincarpos.first;;
		 pt_.y = itr->first.y - miny;// - mincarpos.second;;
		 uc = itr->second;
		 mapout.write((char*)&pt_,sizeof(pt_));
		 mapout.write((char*)&uc,sizeof(uc));
	 }
	 mapout.close();

//	 d<<sizeofmap<<endl;
	 ofstream head("header.txt");
	 head << "the realm of X:" << minx/2.5 << "\t" << maxx/2.5 << "\t" << abs(maxx - minx)/2.5 << endl;
	 head << "the realm of Y :" << miny/2.5 << "\t" << maxy/2.5 << "\t" << abs(maxy - miny)/2.5 << endl;
	 head << "the center point:" <<fixed<<setprecision(1) << (minx + maxx) / 25.0 << "\t" << (miny + maxy) / 25.0 << endl;
	 head << "Starting Point:" << carposvec[0].x - minx << "\t" << carposvec[0].y - miny << endl;
	 head.close();
	// head<<carposmap.size()<<endl;
	 //head <<carposvec.size()<<endl;
	 //head<<"__________________________________________________"<<endl;

	 IplImage * img = cvCreateImage(cvSize(MAPWID, MAPHET), 8, 3);
	 //unsigned char c, c1, c2;
	 unsigned c;
	 for ( int i = 0; i < MAPHET; i++)
		 for (int j = 0; j < MAPWID; j++)
		 {
			 pointT localpoint;
			 localpoint.x = j + minx;
			 localpoint.y = i + miny;
			 c = 127;
			 if (gmap.count(localpoint))
			 {
				 c = gmap[localpoint];
				 if(c<127)
				 {
					 cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(0, 0,0));// cvScalar(g.base, g.road, g.sig));
					 if(c==4)
						 cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(0, 255,0));// cvScalar(g.base, g.road, g.sig));
					 if(c==5)
						 cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(240,128,128));// cvScalar(g.base, g.road, g.sig));
					 if(c==6)
						 cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(0,0,255));// cvScalar(g.base, g.road, g.sig));
					 if(c==0)
						 cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(0,0,0));// cvScalar(g.base, g.road, g.sig));
				 }
				 else
					 cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(255, 0,0));// cvScalar(g.base, g.road, g.sig));
			 }
			 else
				 cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(c, c,c));// cvScalar(g.base, g.road, g.sig));
		 }
		 carposeout.open("carpose.txt");
		 for(int i=0;i<carposvec.size();i++)
		 {
			 // pointT localpoint  = carposvec[i];
			 int m = carposvec[i].y-miny;
			 int n = carposvec[i].x-minx;
			 cvSet2D(img, MAPHET - 1 - m, n, CV_RGB(0, 255,255));// cvScalar(g.base, g.road, g.sig));
			 carposeout<<m<<"\t"<<n<<"\t"<<carposvec[i].eulr<<endl;
		 }
		 carposeout.close();
		 carposeout.close();
		 cvSaveImage("map.png", img);
		 cvReleaseImage(&img);
		// cvWaitKey(0);
}



void LocalMap::SaveFrame(size_t frame)
{
	unsigned short int MAPHET, MAPWID;
	cout<<"Writing the map file...\t\t"<<frame<<endl;
	ofstream mapout("map.gm",ios::binary);
	size_t sizeofmap = gmap.size();
	pointT pt;
	pointT_ pt_;
	unsigned char uc;
	std::string framestr = std::to_string(frame);
	char  filename[100];//= "mappp.gm";
	int minx,miny,maxx,maxy;
	minx = gmap.begin()->first.x;//carpos.x;//car_pose_vec[0].x;
	maxx = gmap.begin()->first.x;//carpos.x;// car_pose_vec[0].x;
	miny = gmap.begin()->first.y;//carpos.y;//car_pose_vec[0].y;
	maxy = gmap.begin()->first.y;//carpos.y;// car_pose_vec[0].y;
	for (map<pointT, unsigned char>::iterator itr = gmap.begin(); itr != gmap.end(); itr++)//for (size_t i = 0; i < local_grid_points_vec.size(); i++)
	{
		if (minx >itr->first.x)
			minx = itr->first.x;
		if (miny> itr->first.y)
			miny = itr->first.y;
		if (maxx <itr->first.x)
			maxx = itr->first.x;
		if (maxy <itr->first.y)
			maxy = itr->first.y;
	}
	MAPWID = abs(maxx - minx);
	MAPHET = abs(maxy - miny);

	// head<<carposmap.size()<<endl;
	//head <<carposvec.size()<<endl;
	//head<<"__________________________________________________"<<endl;

	IplImage * img = cvCreateImage(cvSize(MAPWID, MAPHET), 8, 3);
	//unsigned char c, c1, c2;
	unsigned c;
	for ( int i = 0; i < MAPHET; i++)
		for (int j = 0; j < MAPWID; j++)
		{
			pointT localpoint;
			localpoint.x = j + minx;
			localpoint.y = i + miny;
			c = 127;
			if (gmap.count(localpoint))
			{
				c = gmap[localpoint];
				if(c<127)
				{
					cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(0, 0,0));// cvScalar(g.base, g.road, g.sig));
					if(c==4)
						cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(0, 255,0));// cvScalar(g.base, g.road, g.sig));
					if(c==5)
						cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(210,105,30));// cvScalar(g.base, g.road, g.sig));
					if(c==6)
						cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(0,0,255));// cvScalar(g.base, g.road, g.sig));
				}
				else
					cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(255, 0,0));// cvScalar(g.base, g.road, g.sig));
			}
			else
				cvSet2D(img, MAPHET - 1 - i, j, CV_RGB(c, c,c));// cvScalar(g.base, g.road, g.sig));
		}
		carposeout.open("carpose.txt");
		for(int i=0;i<carposvec.size();i++)
		{
			// pointT localpoint  = carposvec[i];
			int m = carposvec[i].y-miny;
			int n = carposvec[i].x-minx;
			cvSet2D(img, MAPHET - 1 - m, n, CV_RGB(0, 255,255));// cvScalar(g.base, g.road, g.sig));
			carposeout<<m<<"\t"<<n<<"\t"<<carposvec[i].eulr<<endl;
		}
		carposeout.close();
		carposeout.close();
		sprintf(filename, "map_%s.png", framestr.c_str());//,cm);//,cm);
		cvSaveImage(filename, img);
		cvReleaseImage(&img);
	//	cvSaveImage(filename, img);
}

void LocalMap::Process()
{
	cout<<"Num of Rail grids:\t\t"<<RailGridsvec.size()<<endl;

	UpdateMap();
	ShowLocalMap();//FrameNO);

	FrameNO++;
	cout<<"F:"<<FrameNO<<endl;
	//cout<<"car pose set size:"<<carposmap.size()<<endl;
	cout<<m_r.size()<<"\t"<<m_l.size()<<endl;
//	SaveFrame(FrameNO);
	if(FrameNO%300==0 )//����֡�洢һ��//)&&(FrameNO<13203))
	{
		SaveMapWhole();
		//cvWaitKey(0);
	//	getchar();
	}
	//NumOfPointsInGrid.resize(GRID_NUM*GRID_NUM,0);
}



void LocalMap::UpdateMap()
{
	cvZero(localgrid);
	int grid_size = localgrid->width/gridNum;
	long int x,y;
	int row,col;
	pointT localpoint;
	Grid temp_grid;
	carPose car;
	car.x= carpos.x;
	car.y = carpos.y;
	car.eulr =eulr;
	carposvec.push_back(car);
	double S = 0;
	int c;
	for(size_t  i = 0;i<gridNum;i++)// the gray map 
	{
		for (size_t j = 0; j < gridNum; j++)
		{
			row =i;
			col = j;
			c =	P2Color(GridVec[row*gridNum+col].p);
			localpoint.x = carpos.x - (gridNum)/2+col;
			localpoint.y = carpos.y +(gridNum)/2-row;
			if(GridVec[row*gridNum+col].p!=0.5)//&&gmap[localpoint]!=255)
				if(gmap.count(localpoint))
				{
					if(i>abs(shift.y)&&j>abs(shift.x)&&gmap[localpoint]>=0)
					{
						if(c<127)	c =0;
						if(c>127)  c = 255;
						gmap[localpoint ] = c;//P2Color(GridVec[row*gridNum+col].p);
					}
				}
				else //if(GridVec[row*gridNum+col].p!=0.5)//&&gmap[localpoint]!=255)
				{
						if(c<127)	c =0;
						if(c>127)  c = 255;
						gmap[localpoint]  =c;// P2Color(GridVec[row*gridNum+col].p);
				}
		}
	}

	for(int i = 0;i<m_r.size();i++)
	{
		row = m_r[i].y /grid_size;
		col = m_r[i].x /grid_size;
		localpoint.x = carpos.x -(gridNum)/2+col;
		localpoint.y = carpos.y +(gridNum)/2-row;
		if(row>=0&&row<GRID_NUM&&col>=0&&row<GRID_NUM)
				if(curbgrids.count(localpoint))
					curbgrids[localpoint] ++;
				else
					curbgrids[localpoint] = 0;
	}
	for(int i = 0;i<m_l.size();i++)
	{

		row = m_l[i].y /grid_size;
		col = m_l[i].x /grid_size;
		localpoint.x = carpos.x -(gridNum)/2+col;
		localpoint.y = carpos.y +(gridNum)/2-row;
		if(row>=0&&row<GRID_NUM&&col>=0&&row<GRID_NUM)
			if(curbgrids.count(localpoint))
				curbgrids[localpoint] ++;
			else
				curbgrids[localpoint] = 0;
	}

	for( int i = 0;i<RailGridsvec.size();i++)
	{
		row = RailGridsvec[i]/GRID_NUM;
		col =  RailGridsvec[i]%GRID_NUM;
		localpoint.x = carpos.x - (gridNum)/2 + col;
		localpoint.y = carpos.y + (gridNum)/2 - row;
		Railgrids.insert(localpoint);
	}
	for(int i= 0;i<NumOfPointsInGrid.size();i++)
	{
		row = i/GRID_NUM;
		col = i%GRID_NUM;
		localpoint.x = carpos.x - (gridNum)/2 + col;
		localpoint.y = carpos.y + (gridNum)/2 - row;
		if(!Trunk.count(localpoint)&&NumOfPointsInGrid[i]>20)
			Trunk.insert(localpoint);
	}

	for(map<pointT,unsigned short int> ::iterator itr = curbgrids.begin();itr!=curbgrids.end();itr++)
		if(itr->second>0)
			if(gmap[itr->first]!=255)
				gmap[itr->first] = 4;

	for(set<pointT>::iterator itr= Railgrids.begin();itr!=Railgrids.end();itr++)
		if(gmap[*itr]!=255&&gmap[*itr]!=4)
			gmap[*itr] = 5;

	for(set<pointT>::iterator itr=Trunk.begin();itr!=Trunk.end();itr++)
		if(gmap[*itr]!=255)
			gmap[*itr] = 6;
}


int LocalMap::P2Color(double p)
{
	int delta = 70;
	if(p<0.5&& p>=0)
		return 255*(1-p);

	if(p>0.5&&p<=1)
		return 127- (p-0.5)/0.05*delta >=0?127- (p-0.5)/0.05*delta:0;

	return 127;
}
