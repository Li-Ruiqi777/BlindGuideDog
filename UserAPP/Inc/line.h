#ifndef __Line__H
#define __Line__H
#include "opencv.hpp"
#include <vector>
using namespace std;
using namespace cv;

class Line
{
public:
	typedef enum
	{
		Normal = 0,	//完整的线在图片内
		Border,			//线的一部分不在图片内
	}State_t;

	Line(void) :_color(Scalar(0, 0, 0)) {}
	Line(Scalar color) :_color(color) {}
	~Line() {}

	void fitting(void);
	void fitting_BEV(const Mat& H);
	void draw(Mat& scrImage);
	double angle = 0;		//直线与y轴正方向的夹角(度)
	double k = 0;				//直线斜率
	vector<Point>* pts = nullptr; //直线上的离散点
	Point mid;					//拟合出的直线的中点
	Scalar _color;			//画图时用的颜色
	State_t sta = Normal;
};

void rad2degree(double& angle);
void degree2rad(double& angle);
Point2d Get_P_World(const Mat& H, Point2d P_pixel);
vector<Point2d> Get_Pts_World(const Mat& H, vector<Point> Pts_pixel);

#endif