#include "config.h"
#include "defines.h"
#include "HDLDisplay.h"
#include <iostream>
//#include "CurbScanner.h"

using namespace std;
//ofstream outeulr("eulr.txt'");

HDLDisplay::HDLDisplay(void)
{
	existleft=false,existright=false;
	nowcount=0;sum=0;veiy=0;
	d_map = new DynamicMap;
	s_map = new StaticMap;
	l_map  = new LocalMap;
	m_curb = new CurbScanner();

	for (int i=0;i<MAX_TIMES;++i) pointcount[i]=0;
	m_hdldata = NULL;
	m_veidata = NULL;
	m_out = NULL;
	m_save_cloud = NULL;
	m_size = cvSize(500, 500);
	img = cvCreateImage(cvSize(1000,1000),8,1);
	allimg = cvCreateImage(m_size,8,1);
	image= cvCreateImage(cvSize(1000,1000),8,1);
	mulImage = cvCreateImage(cvSize(1000,1000),8,1);
	tim  = cvCreateImage(cvSize(1000,1000), IPL_DEPTH_8U, 1);
	xim  = cvCreateImage(cvSize(1000,1000), IPL_DEPTH_8U, 3);
	trans = cvCreateMat (2, 3, CV_32FC1); 
	nCount = 0;
    gridsize =1000.0/GRID_NUM;
    grid_size =((20.0)/GRID_NUM)/**2.5*/;
	m_curb->Initialize();
	//PointsInGrid.resize(GRID_NUM*GRID_NUM, make_pair(0, 0));
	NumOfPointsInGrid.resize(GRID_NUM*GRID_NUM,0);//NumOfPoints.rend(GRID_NUM*GRID_NUM,0);
}

vector<CvPoint> HDLDisplay::Convert(vector<CvPoint> pts,double eulr)
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
vector<CvPoint> HDLDisplay::Tranform( vector<CvPoint> pts)
{
	vector<CvPoint> result;
	CvPoint tp;
	for(size_t i = 0;i<pts.size();i++)
	{
		tp.x = pts[i].x -m_size.width;
		tp.y = m_size.height - pts[i].y;
		if(pts[i].y<250)
		result.push_back(tp);
	}
	return result;
}
//
vector<CvPoint> HDLDisplay::ReTranform( vector<CvPoint> pts)
{
	vector<CvPoint> result;
	CvPoint tp;
	for(size_t i = 0;i<pts.size();i++)
	{
		tp.x = pts[i].x+m_size.width;
		tp.y = m_size.height - pts[i].y;
		result.push_back(tp);
	}
	return result;
}

HDLDisplay::~HDLDisplay(void)
{
	if (m_hdldata)
	{
		fflush(m_hdldata);
		fclose(m_hdldata);
		m_hdldata = NULL;
	}
	if (m_veidata)
	{
		fflush(m_veidata);
		fclose(m_veidata);
		m_veidata = NULL;
	}
	/*if (m_out)
	{
		fflush(m_out);
		fclose(m_out);
		m_veidata = NULL;
	}*/

	if (m_timestampdata)
	{
		m_timestampdata.close();
	}
	if (m_save_cloud)
	{
		delete[] m_save_cloud;
		m_save_cloud = NULL;
	}
	cvReleaseImage(&img);
	cvReleaseImage(&allimg);
	cvReleaseImage(&image);
	cvReleaseImage(&mulImage);
	cvReleaseImage(&tim);
	cvReleaseMat(&trans);
	cvDestroyWindow("img");
	cvDestroyWindow("allimg");
}

bool HDLDisplay::Initialize(const char* hdlfilepath)
{
	m_save_cloud = new PointSave[CLOUD_NUM];
	m_save_cloud_count = 0;

	for (int i = 0; i < LASER_NUM; i++)
	{
		m_layer_cloud[i].c = 0;
	}
    bool result = load_laser_info("new_xml.txt");
    if (!result){
        LOG(FATAL) << "Couldn't find laser info file \"new_xml.txt\"\n";
		return false;
    }
    result = (m_hdldata = fopen(hdlfilepath, "rb"));
    if ((m_hdldata = fopen(hdlfilepath, "rb")) == NULL)
    {
        LOG(FATAL) << "read hdl file " << hdlfilepath << " error!" << std::endl;
        return false;
    }

	char vpath[100]={0};strcpy(vpath,hdlfilepath);
	vpath[strlen(hdlfilepath)-3]='\0';strcat(vpath,"hdl_dgps");
	if ((m_veidata = fopen(vpath, "r")) == NULL)
	{
        LOG(FATAL) << "read vehicle file " << vpath << " error!" << endl;
		return false;
	}

/*	if ((m_out = fopen("out.txt", "w")) == NULL)
	{
		cout << "write angle file " << hdlfilepath << " error!" << endl;
		return false;
	}*/
	
	return true;
}

//read the fixed info for each laser
bool HDLDisplay::load_laser_info(const char* data_path)
{
	//the dat file is converted from db.xml, only have the array info
	//used parse_xml_db solution to convert xml to dat file
	m_cos_raw = new float[ANGLE_NUM];             // ANGLE_NUM = 36000
	m_sin_raw = new float[ANGLE_NUM];
	ifstream fdb(data_path);
	if (!fdb)
	{
		cout << "read laser info file error!" << endl;
		return false;
	}

	for (int i = 0; i < LASER_NUM; i++)
	{
		rasersort[i]=i;
		fdb >> m_rot[i] >> m_vert[i] >> m_dist[i] >> m_z_off[i] >> m_x_off[i] >> m_min_i[i] >> m_max_i[i] >> m_distX[i] >> m_distY[i] >> m_f_d[i] >> m_f_s[i];
	}
	fdb.close();
	

	for (int i=0;i<LASER_NUM-1;i++)
		for (int j=i+1;j<LASER_NUM;j++)
			if (m_vert[rasersort[i]]>m_vert[rasersort[j]])
			{
				int ts=rasersort[i];rasersort[i]=rasersort[j];rasersort[j]=ts;
			}
	for (int i=0;i<LASER_NUM;i++) layermark[rasersort[i]]=i;
	
	//regulate unit to mm
	for (int i = 0; i < LASER_NUM; i++)
	{
		m_dist[i]  *= 10;m_z_off[i] *= 10;m_x_off[i] *= 10;m_distX[i] *= 10;m_distY[i] *= 10;
	}

	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_cos_rot[i] = cos( m_rot[i] / 180.0f * M_PI);
		m_sin_rot[i] = sin( m_rot[i] / 180.0f * M_PI);
		m_cos_vert[i] = cos( m_vert[i] / 180.0f * M_PI);
		m_sin_vert[i] = sin( m_vert[i] / 180.0f * M_PI);
	}

	for (int i = 0; i < ANGLE_NUM; i++)
	{
		m_cos_raw[i] = cos( i / 18000.0f * M_PI);
		m_sin_raw[i] = sin( i / 18000.0f * M_PI);
	}

	return true;
}

void HDLDisplay::comput(double &orix,double &oriy,double &orit)
{
	int last1=(nowcount+MAX_TIMES-1)%MAX_TIMES;
	int last2=(nowcount+MAX_TIMES-2)%MAX_TIMES;
	double dis=sqrt((savex[last1]-savex[last2])*(savex[last1]-savex[last2])
					+(savey[last1]-savey[last2])*(savey[last1]-savey[last2]));
	//dis0 is the transition of the vehicle
	double dis0=sqrt(veix*veix+veiy*veiy);
	orit=2*savet[last1]-savet[last2];
	if (dis==0)
	{
		orix=savex[last1];oriy=savey[last1];
	}else{
		orix=savex[last1]+(savex[last1]-savex[last2])*(dis-dis0)/dis;
		oriy=savey[last1]+(savey[last1]-savey[last2])*(dis-dis0)/dis;
	}
	return ;
}
	
bool HDLDisplay::Process()
{
	//struct timeval start, end;
	//gettimeofday(&start, NULL);
 	unsigned short rot;
	unsigned char c;
	sum++;
	unsigned char multply_ratio = 25;
	unsigned char ic;
	double cx,cy;
	double dx,dy;
	int col, row;
	int cc,rr;
	int id;
	pair<int,int> temp;
	int dis;
	unsigned short dist;


	if ((int)fread(&m_save_cloud_count, sizeof(int), 1, m_hdldata) < 1) return false;
	cout<<"m_save_cloud_count: "<<m_save_cloud_count<<endl;
	
//	if ((int)fread(m_cloud.pts, sizeof(Points), m_save_cloud_count, m_hdldata) < m_save_cloud_count) return false;
	//read a frame of point cloud into m_svae_cloud
	if ((int)fread(m_save_cloud, sizeof(PointSave), m_save_cloud_count, m_hdldata) < m_save_cloud_count) return false;
	m_cloud.pts_count = m_save_cloud_count;

	lastveiy=veiy;
	//read car position of current frame
	if ((int)fscanf(m_veidata,"%lf %lf %lf",&veix,&veiy,&veit) < 3) return false;
	Pose_t tmpp;
	tmpp.x  = veix;
	tmpp.y = veiy;
	tmpp.eulr  = veit;
	m_vehicle_pos_t.push_back(tmpp);
	//only when nCount>210 do the process begin. Note: is if-statement last nearly to the end of this function.
	if(nCount>210)//1400
	{
	eulr = veit;
	//outeulr<<eulr<<endl;
	//if(eulr>0)
	//	eulr = eulr -2*M_PI; 
	d_map->TrafficSignGrid.clear();
	Railgridsvec.clear();
	NumOfPointsInGrid.resize(GRID_NUM*GRID_NUM,0);
	cvZero(img);
	//this for loop process one point a time.
	for (int i = 0; i < m_save_cloud_count; i++)
	{
		ic = m_cloud.pts[i].i = m_save_cloud[i].i;//laser point intensity
		dist =m_cloud.pts[i].dist = m_save_cloud[i].dist;//distance
		c = m_cloud.pts[i].c = m_save_cloud[i].c;//
		rot = m_cloud.pts[i].rot = m_save_cloud[i].rot;

		float cos_phi = m_cos_vert[c];
		float sin_phi = m_sin_vert[c];
		float cos_theta = m_cos_raw[rot] * m_cos_rot[c] + m_sin_raw[rot] * m_sin_rot[c];
		float sin_theta = m_sin_raw[rot] * m_cos_rot[c] - m_cos_raw[rot] * m_sin_rot[c];
        float r1 = m_save_cloud[i].dist * 2.0f;
		float r = r1 + m_dist[c];

		float rxy = r * cos_phi;
		float xx = abs(rxy * sin_theta - m_x_off[c] * cos_theta);
		float yy = abs(rxy * cos_theta + m_x_off[c] * sin_theta);

		float rx = (m_dist[c] - m_distX[c]) * (xx/22640.0f - 0.106007f) + m_distX[c];
		float ry = (m_dist[c] - m_distY[c]) * (yy/23110.0f - 0.083514f) + m_distY[c];

		//x:
		r = r1 + rx;
		rxy = r * cos_phi;
		int x = (int)(rxy * sin_theta - m_x_off[c] * cos_theta);

		//y:
		r = r1 + ry;
		rxy = r * cos_phi;
		int y = (int)(rxy * cos_theta + m_x_off[c] * sin_theta);

		//z:
		r = r1 + m_dist[c];
		int z = (int)(r * sin_phi + m_z_off[c]);

		m_cloud.pts[i].x = x;
		m_cloud.pts[i].y = y;
		m_cloud.pts[i].z = z;
		dx =(double) x/1000.0;
		dy=(double) y/1000.0;
		cx =dx * cos(eulr) + dy * sin(eulr);
		cy =dy* cos(eulr) - dx* sin(eulr);

		//cx = x * cos(eulr) + y * sin(eulr);
		//cy = y* cos(eulr) - x* sin(eulr);
		col = (float)((cx - m_scale.x_min) * m_scale.x_scale)+m_size.width/2;
		row = m_size.height - (float)((cy - m_scale.y_min) * m_scale.y_scale);
		 if( dist>2000)
		cvCircle(img,cvPoint(col,row),0,CV_RGB(255,255,255),-1);
		cc= (float)col/gridsize;
		rr = (float)row/gridsize;
		id =rr*GRID_NUM + cc;
		//if the intensity (ic) is greater than 200 and x less than 7000, and its hight is greater than 0, then it's a traffic sign.
		if (ic>200&&z>0 &&abs(x)<7000)
			if (d_map->TrafficSignGrid.count(id))
				d_map->TrafficSignGrid[id]++;
			else
				d_map->TrafficSignGrid[id] = 1;

			//if blahblah..., then the point is a rail point.
			if(z>0&&id<GRID_NUM*GRID_NUM&&cc>=0&&cc<GRID_NUM&&rr>=0&&rr<GRID_NUM&&abs(x)<10000&&ic>50)//&&abs(PointsInGrid[id].first - PointsInGrid[id].second)>500)//cc<1500&&rr>0&&rr<1500)
				Railgridsvec.push_back(id);

			//if abs(x)<10000, then the point is valid??
			if(id<GRID_NUM*GRID_NUM&&cc>=0&&cc<GRID_NUM&&rr>=0&&rr<GRID_NUM&&abs(x)<10000)//&&abs(PointsInGrid[id].first - PointsInGrid[id].second)>500)//cc<1500&&rr>0&&rr<1500)
				NumOfPointsInGrid[id] ++;//	Railgridsvec
	}





#if 1

	if (lastveiy-100>veiy)
	{
		double orix,oriy,orit,u,v;comput(orix,oriy,orit);
		for (int i=0;i<MAX_TIMES;i++)
			if (pointcount[i]!=0)
			{
				u=(savex[i]-orix)*cos(orit)-(savey[i]-oriy)*sin(orit);
				v=(savex[i]-orix)*sin(orit)+(savey[i]-oriy)*cos(orit);
				savet[i]-=orit;savex[i]=u;savey[i]=v;
			}
	}
	if (pointcount[nowcount]!=0)
		{farlastx=savex[nowcount];farlasty=savey[nowcount];farlastt=savet[nowcount];}
	else 
		{farlastx=savex[0];farlasty=savey[0];farlastt=savet[0];}

	if(nCount)
		//multply_ratio is set to 25.
		delta = cvPoint(int(((m_vehicle_pos_t[nCount].x)-(m_vehicle_pos_t[nCount-1].x))*multply_ratio)
		,int(((m_vehicle_pos_t[nCount].y)-(m_vehicle_pos_t[nCount-1].y))*multply_ratio));
	else
		delta = cvPoint(0,0);
	m_curb->m_cloud_count = m_save_cloud_count;
	memcpy(m_curb->m_cloud, m_cloud.pts, sizeof(LPoint_t) * m_save_cloud_count);
	//cout<<m_save_cloud_count<<endl;
	m_curb->ProcessFrame();
	//cout<<"curb line:\t\t\t"<<endl;
	//cout<<"size:\t\t\t"<<m_curb->m_l.size()<<"\t"<<m_curb->m_r.size()<<endl;
	m_r.clear();
	m_l.clear();
	m_r = ReTranform(Convert(Tranform(m_curb->m_r),eulr));
	m_l = ReTranform(Convert(Tranform(m_curb->m_l),eulr));
	//cvWaitKey(0);
	cvZero(image);

	//getchar();
//	m_hdl->Process(m_curb->m_r,m_curb->m_l);

	if (ransac())
	{
		filter();
		ishow();
		ishowall();
		char tttt=cvWaitKey(1);if (tttt==32) tpause=0;
	//	cvWaitKey(0);
	}
}
	//NumOfPointsInGrid.clear();
	//for( int i =0;i<m_r.size();i++)
	//	cvCircle(image,cvPoint(m_r[i].x,m_r[i].y),3,CV_RGB(5,255,5),-1);
	//cvShowImage("imgmgmgm",image);
	//	cvShowImage("img",img);
//cvShowImage("img",img);
//for(int i= 0;i<NumOfPointsInGrid.size();i++)
//{
//	row = i/GRID_NUM;
//	col = i%GRID_NUM;
//	if(NumOfPointsInGrid[i]>5)
//		cvRectangle(img,cvPoint(col*gridsize ,row*gridsize)	,cvPoint((col+1)*gridsize ,(row+1)*gridsize),CV_RGB(0,0,255),-1);
//}
NumOfPointsInGrid.clear();
//cvShowImage("img",img);

#endif
	nCount++;
	cout<<"nCount:\t"<<nCount<<endl;
	return true;
}


void HDLDisplay::ishowall()
{
	cvmSet(trans,0,0,1);
	cvmSet(trans,0,1,0);
	cvmSet(trans,0,2,-delta.x);
	cvmSet(trans,1,0,0);
	cvmSet(trans,1,1,1);
	cvmSet(trans,1,2,delta.y);
	cvZero(tim);
	cvWarpAffine(mulImage,tim,trans);
	cvAdd(image,tim,tim);

	cvZero(mulImage);
	cvCopy(tim,mulImage);
	// cvShowImage("multiple img",mulImage);
}

bool HDLDisplay::ransac()
{
	int i,j,dx,dy,dz,func,dist,rot;

	unsigned int v[LASER_NUM]={0};
	long xsum=0,ysum=0,zsum=0,a[9]={0},efcount=0,countmax=0,count=0;
	CvMat* M = cvCreateMat(3,3,CV_32FC1);
	CvMat* E = cvCreateMat(3,3,CV_32FC1);
    CvMat* I = cvCreateMat(3,1,CV_32FC1);
	csan=0;
	
	for (i = 0; i < m_cloud.pts_count; i++)
	{
		dx=m_cloud.pts[i].x;dy=m_cloud.pts[i].y;dz=m_cloud.pts[i].z;
		dist = m_cloud.pts[i].dist;
		if ( dz<=-1100 &&  dy>=1000 && dy<30000 && dx>=-2000 && dx<=2000)// &&dist>15000)
		{
			psan[csan].x=dx;psan[csan].y=dy;psan[csan].z=dz;
			psan[csan].c=rasersort[m_cloud.pts[i].c];csan++;
		}
	}
	efcount=csan;
	for (func=0;func<efcount/50;func++)
	{
		int ranp[10];csan=10;
		for (i=0;i<csan;i++) ranp[i]=rand()%efcount;
		long rxsum=0,rysum=0,rzsum=0,a[9]={0},tempcount=0;
		for (i=0;i<csan;i++) {rxsum+=psan[ranp[i]].x;rysum+=psan[ranp[i]].y;rzsum+=psan[ranp[i]].z;}
		rxsum/=csan;rysum/=csan;rzsum/=csan;
		for (i=0;i<csan;i++)
		{
			dx=psan[ranp[i]].x-rxsum;dy=psan[ranp[i]].y-rysum;dz=psan[ranp[i]].z-rzsum;
			a[0]+=dx*dx;a[1]+=dx*dy;a[2]+=dx*dz;
			a[3]+=dx*dy;a[4]+=dy*dy;a[5]+=dy*dz;
			a[6]+=dx*dz;a[7]+=dy*dz;a[8]+=dz*dz;
		}
    	cvZero(M);cvZero(E);cvZero(I);
		for (i=0;i<9;i++) cvmSet(M,i/3,i%3,a[i]);
		cvEigenVV(M, E, I);
		double rsana=cvmGet(E,2,0),rsanb=cvmGet(E,2,1),rsanc=cvmGet(E,2,2);
		double rsand=rsana*rxsum+rsanb*rysum+rsanc*rzsum;
		for (i=0;i<efcount;i++)
			if (absd(psan[i].x*rsana+psan[i].y*rsanb+psan[i].z*rsanc-rsand)<=50) tempcount++;
		if (tempcount>countmax)
		{
			countmax=tempcount;sana=rsana;sanb=rsanb;sanc=rsanc;sand=rsand;
		}
	}

	//cout<<"Accuracy: "<<(double)countmax/(double)efcount*100<<"% Iteration times: "<<func<<" Points:"<<efcount<<endl;
	if (countmax*10<efcount*3) return false;csan=efcount;
	cvReleaseMat(&M);cvReleaseMat(&E);cvReleaseMat(&I);
	for (i=0;i<m_cloud.pts_count;i++)
	{
		dx=m_cloud.pts[i].x;dy=m_cloud.pts[i].y;dz=m_cloud.pts[i].z;
		dist = m_cloud.pts[i].dist;
	//	ic = m_cloud.pts[i].i;
		if ( dz <= -1100 && dy >0 && dy<30000 && dx>-15000 && dx<15000)// &&dist>2000)
		if (absd(dx*sana + dy*sanb + dz*sanc - sand) <= 100) 
		{
			unsigned char c = m_cloud.pts[i].c;
			m_cloud.pts[count].c 	= c;
			m_cloud.pts[count].x 	= dx;
			m_cloud.pts[count].y 	= dy;
			m_cloud.pts[count].z 	= dz;
			m_cloud.pts[count].i 	= m_cloud.pts[i].i;
			m_cloud.pts[count].dist = dist;
			m_layer_cloud[c].p[v[c]] = count;
			v[c]++;count++;
		}
		//dx/=1000;
		//dy/=1000;
		//cx =dx * cos(eulr) + dy * sin(eulr);
		//cy =dy* cos(eulr) - dx* sin(eulr);
		//col = (int)((cx - m_scale.x_min) * m_scale.x_scale)+m_size.width/2;
		//row = m_size.height - (int)((cy - m_scale.y_min) * m_scale.y_scale);
		//c= col/gridsize;
		//r = row/gridsize;
		//id =r *GRID_NUM + c;

		//if (ic>200&&ic<255 && dz>0 && dz<1000)//&&abs(y)<5000)
		//	if (d_map->TrafficSignGrid.count(id))
		//		d_map->TrafficSignGrid[id]++;
		//	else
		//		d_map->TrafficSignGrid[id] = 1;
	}
	m_cloud.pts_count=count;
	for (int i = 0; i < LASER_NUM; i++) m_layer_cloud[i].c = v[i];
	return true;
}


void HDLDisplay::ishow()
{
	int col, row;
	int c,r;
	int id;
	unsigned char ic;
	unsigned int dist;
	double x, y;	
	double cx,cy;
	int z;
	cvZero(image);
	cvZero(xim);
	d_map->PointsInGrid.clear();
	//if(nCount%20==0)
	//	cvWaitKey(0);
	//ofstream out1("out1.txt"),out2("out2.txt");
	for (int i = 0; i < m_cloud.pts_count; i++)//for (int i = 0; i < m_cloud.pts_count; i++)
	{
		x = (double)m_cloud.pts[i].x / 1000;
		y = (double)m_cloud.pts[i].y / 1000;
		dist = m_cloud.pts[i].dist;
		cx = x * cos(eulr) + y * sin(eulr);
		cy = y* cos(eulr) - x* sin(eulr);
		col = (int)((cx - m_scale.x_min) * m_scale.x_scale)+m_size.width/2;
		row = m_size.height - (int)((cy - m_scale.y_min) * m_scale.y_scale);
		c= col/gridsize;
		r = row/gridsize;
		id =r *GRID_NUM + c;
	
	if (col >= 0 && col < m_size.width *2&& row >= 0 && row < m_size.height*2)//&&x>-3)
			if(x>-3&&x<3)
			{
				if((m_cloud.pts[i].c<62)&&(m_cloud.pts[i].c!=54)&&(m_cloud.pts[i].c!=48)&&(m_cloud.pts[i].c!=50)&&(m_cloud.pts[i].c!=38)&&(m_cloud.pts[i].c!=36))//&&(x>-3)//	||(x<-3)&&(m_cloud.pts[i].c!=37)&&(m_cloud.pts[i].c!=39)&&(m_cloud.pts[i].c!=33)&&(m_cloud.pts[i].c!=49)&&(m_cloud.pts[i].c!=40)&&(m_cloud.pts[i].c!=32))//&&()//if(!(m_cloud.pts[i].c<34&&m_cloud.pts[i].c>=32||m_cloud.pts[i].c>35&&m_cloud.pts[i].c<44))
				if(m_cloud.pts[i].i ==255)
				{
					image->imageData[row * m_size.width *2+ col] = 255;
					if(d_map->PointsInGrid.count(id))
						d_map->PointsInGrid[id]++;
					else
						d_map->PointsInGrid[id] = 1;
				}
			}
			else
			{
				if((m_cloud.pts[i].c<62)&&(m_cloud.pts[i].c!=54)&&(m_cloud.pts[i].c!=48)&&(m_cloud.pts[i].c!=50)&&(m_cloud.pts[i].c!=38)&&(m_cloud.pts[i].c!=36)
				  &&(m_cloud.pts[i].c!=37)&&(m_cloud.pts[i].c!=39)&&(m_cloud.pts[i].c!=33)&&(m_cloud.pts[i].c!=49)&&(m_cloud.pts[i].c!=40)&&(m_cloud.pts[i].c!=32))//&&()//if(!(m_cloud.pts[i].c<34&&m_cloud.pts[i].c>=32||m_cloud.pts[i].c>35&&m_cloud.pts[i].c<44))
						if(m_cloud.pts[i].i ==255)
						{
							image->imageData[row * m_size.width *2+ col] = 255;
							if(d_map->PointsInGrid.count(id))
								d_map->PointsInGrid[id]++;
							else
								d_map->PointsInGrid[id] = 1;
						}
			}

		if (ic>200&&ic<255 && z>0 && z<1000)//&&abs(y)<5000)
			if (d_map->TrafficSignGrid.count(id))
				d_map->TrafficSignGrid[id]++;
			else
				d_map->TrafficSignGrid[id] = 1;

	}
	//cout<<"Traffic Sign:\t"<<d_map->TrafficSignGrid.size()<<endl;

	ProcessDynamicMap();
	ProcessStaticMap();
	ProcessLocalMap();
}



void HDLDisplay::filter()
{
	int i,j,u,v,count=0;
	unsigned int k;
	float tmp,layer_intense[4500];
	csan=0;
	//ofstream out1("out1.txt");ofstream out2("out2.txt");
	for (  i = 0; i < LASER_NUM; i++)
	{
		if (m_layer_cloud[i].c > 0)
		{
			k = m_layer_cloud[i].p[0];
			for ( j = 0; j < filter_length; j++) layer_intense[j] = m_cloud.pts[k].i;
			for ( j = 0; j < m_layer_cloud[i].c; j++)
			{
				k = m_layer_cloud[i].p[j];
				layer_intense[j + filter_length] = m_cloud.pts[k].i;
			}

			k = m_layer_cloud[i].p[m_layer_cloud[i].c - 1];
			for ( j = m_layer_cloud[i].c + filter_length; j < m_layer_cloud[i].c + filter_length*2; j++) layer_intense[j] = m_cloud.pts[k].i;
	
			
			for( u = 0; u < m_layer_cloud[i].c + filter_length; u++)
			{
				tmp = 0.0;
				for(v = 0; v < filter_length; v++)
				{
					tmp += float(layer_intense[u + filter_length - v]) * f[v];
				}
				
				if  ( u >=  filter_length/2 && u < (m_layer_cloud[i].c + filter_length/2)  )
				{
					k = m_layer_cloud[i].p[u - filter_length/2 ];
					if( tmp >= 50  || tmp >= 15-(int)(m_vert[i]*4))
					{
						//out1 << m_cloud.pts[k].x << endl;
						//out2 << m_cloud.pts[k].y << endl;
						int x=m_cloud.pts[k].x,y=m_cloud.pts[k].y;
						if (tmp >= 50)
						{
							//savep[nowcount][count].x=(int)(cos(veit)*x+sin(veit)*y+veix*1000);
							//savep[nowcount][count].y=(int)(-sin(veit)*x+cos(veit)*y+veiy*1000);
							savep[nowcount][count].x=x;savep[nowcount][count].y=y;
							savep[nowcount][count].c=m_cloud.pts[k].c;count++;
							m_cloud.pts[k].i = 255;
						}else
							m_cloud.pts[k].i = 0;
						psan[csan].x=x;psan[csan].y=y;
						psan[csan].z=m_cloud.pts[k].i;
						psan[csan].c=m_cloud.pts[k].c;
						csan++;
					}
					else
						m_cloud.pts[k].i = 0;
				}
				
			}
		}

	}
	pointcount[nowcount]=count;
	//out1.close();out2.close();
}


void HDLDisplay::ProcessDynamicMap()
{
	d_map->delta = delta;
	d_map->eulr =m_vehicle_pos_t[nCount].eulr ;
	d_map->Process();
} 


void HDLDisplay::ProcessStaticMap()
{
	s_map->shift = cvPoint(int((m_vehicle_pos_t[nCount].x)/grid_size)-int((m_vehicle_pos_t[nCount-1].x)/grid_size)
		,int((m_vehicle_pos_t[nCount].y)/grid_size)-int((m_vehicle_pos_t[nCount-1].y)/grid_size));
	s_map->eulr = d_map->eulr;
	s_map->dynamicgrid = d_map->gridmap;
	s_map->gridNum = d_map->gridNum;
	s_map->DynamicGridVec = d_map->MeasurementGrid;
	s_map->carpos.x = int((m_vehicle_pos_t[nCount].x)/grid_size);
	s_map->carpos.y = int((m_vehicle_pos_t[nCount].y)/grid_size);//cout<<"Moved:\t"<<s_map->shift.x<<"\t"<<s_map->shift.y<<endl;
	s_map->Process();
}

void HDLDisplay::ProcessLocalMap()
{
//	cout<<"static vec size:\t"<<s_map->StaticGridVec.size()<<endl;
	//l_map->Railgrids = Railgrids;
	l_map->RailGridsvec = Railgridsvec;
	l_map->NumOfPointsInGrid = NumOfPointsInGrid;
	l_map->GridVec = s_map->StaticGridVec;
	l_map->shift = s_map->shift;
	l_map->carpos.x = int((m_vehicle_pos_t[nCount].x)/grid_size);
	l_map->carpos.y = int((m_vehicle_pos_t[nCount].y)/grid_size);
	l_map->eulr = d_map->eulr;
	l_map->m_l = m_l;
	l_map->m_r = m_r;
	l_map->Process();
}
