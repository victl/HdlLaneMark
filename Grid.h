#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <set>
using namespace cv;
enum GridType {car_track,boundary,occupied,unoccupied,right_boundary,left_boundary};

struct Grid{
	//unsigned char num_of_points;
	float p;
	//GridType gt;
	//unsigned short delta_z;
};

struct G
{
	unsigned char p;
	unsigned char type;
	//int x, y;
}; 

struct pointT
{
	int x,y;
	bool operator < (const pointT other) const
	{
		if(x!=other.x)
			return x<other.x;
		return y<other.y;
	}
};
struct carPose
{
	int x,y;
	float eulr;
};


struct pointT_
{
	unsigned short int x,y;
	bool operator < (const pointT other) const
	{
		if(x!=other.x)
			return x<other.x;
		return y<other.y;
	}
};

template < class T >
void ClearVector( vector< T >& vt ) 
{
	vector< T > ().swap( vt );
}
