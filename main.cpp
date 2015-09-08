#include "config.h"
#include "defines.h"
#include "HDLDisplay.h"
#include <stdio.h>
#include <iostream>
#include "config.h"

using namespace std;

HDLDisplay *m_hdl = NULL;
bool tpause;

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		cout << "Usage: lanemark 'hdl_file' 'output_dir'" << endl;
		return -1;
	}
	m_hdl = new HDLDisplay();
//	argc = 1;
	//if (argc == 1)
	//{
	//	if (!m_hdl->Initialize("E:/HDL/20130926_105321.hdl")) return 1;
	//	if (!m_hdl->Initialize("D:/LUXDATA/hdldata/20130926_105321.hdl")) return 1;//D:\LUXDATA\hdldata
	//if (!m_hdl->Initialize("D:/LUXDATA/hdldata/20131010_162539.hdl")) return 1;//20131010_162539
	//if (!m_hdl->Initialize("D:/LUXDATA/hdldata/20131010_162543.hdl")) return 1;//20131010_162543
	//if (!m_hdl->Initialize("D:/LUXDATA/hdldata/20131010_162627.hdl")) return 1;//20131010_162627
	//if (!m_hdl->Initialize("D:/LUXDATA/hdldata/20131010_162630.hdl")) return 1;//20131010_162630
	//if (!m_hdl->Initialize("D:/LUXDATA/hdldata/20131013_132151.hdl")) return 1;//20131013_132151//!!!!!!!!!!!!!!!!!!!!!!!!
	//if (!m_hdl->Initialize("D:/LUXDATA/hdldata/20131010_162539.hdl")) return 1;//20131010_162539
	//if (!m_hdl->Initialize("D:/LUXDATA/hdldata/20131010_162539.hdl")) return 1;//20131010_162539
	//if (!m_hdl->Initialize("D:/hdl/n1.hdl")) return 1;//20131010_162539///////////this is it!!!!!!!!!!!!!!!//D:/hdl/20141106_052925
	//if (!m_hdl->Initialize("E:/HDL1106/20141106_052925.hdl")) return 1;
	//	if (!m_hdl->Initialize("D:/LUXDATA/hdldata/n2.hdl")) return 1;//20131010_162539///this is it!!!!!!!!!!!!!!!
	//D:/hdl/20141022_150940
	//	if (!m_hdl->Initialize("E:/HDL1022/20141022_150940.hdl")) return 1;

	//if (!m_hdl->Initialize("E:/HDL1031/20141031_220200.hdl")) return 1;//9����//1382
	//if (!m_hdl->Initialize("E:/HDL1031/20141103_205543.hdl")) return 1;//9����//20141031_220200
	//if(1) { if (!m_hdl->Initialize("E:/HDL1031/20141031_220838.hdl")) return 1;}//�ļ�·��//9����//13827		//if (!m_hdl->Initialize("E:/HDL1031/20141031_220200.hdl")) return 1;//9����//1382�ļ�·��
//	if(1) { if (!m_hdl->Initialize(argv[1])) return 1;}//�ļ�·��//9����//13827		//if (!m_hdl->Initialize("E:/HDL1031/20141031_220200.hdl")) return 1;//9����//1382�ļ�·��
	//else {  if (!m_hdl->Initialize("E:/HDL1031/20141103_205543.hdl")) return 1;}//9����//20141031_220200

	//if (!m_engine->OpenHDLFile("E:/HDL0811/20140813_154747")) return -1;//E:\hdld//1000-1500//10350-10800
//	if (!m_hdl->Initialize("E:/HDL0811/20140813_154747.hdl")) return 1;

//20141031_152227
	//	cout<<" argc  1"<<endl;
		//cvWaitKey(0);
	//}else
	//{
	//	char namp[100]={0};strcpy(namp,"E:/HDL1106/201416_052925");
	//	strcat(namp,argv[1]);strcat(namp,".hdl");
	//	if (!m_hdl->Initialize(namp)) return 1;
	//	cout<<" argc  2"<<endl;
	//	cvWaitKey(0);
    //}
    if (!m_hdl->Initialize(argv[1])) {
        DLOG(INFO)<<"error initialize HDLDisplay";
        return 1;
    }
	int  frame=0;
	//cvWaitKey(0);
	//getchar();
	bool flag=true;tpause=true;
	while (flag)
	{
		frame++;
		if (tpause) flag = m_hdl->Process();
		else {
			char tttt=cvWaitKey(1);
			if (tttt==32) tpause=true;
			else if (tttt==27) break;
		}
	//	cout<<frame<<endl;;
	}
	delete m_hdl;
	m_hdl = NULL;
	return 0;
}
