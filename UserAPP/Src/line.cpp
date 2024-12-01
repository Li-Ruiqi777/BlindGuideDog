#include "line.h"

void rad2degree(double& angle)
{
	angle = angle * 180 / 3.14;
}

void degree2rad(double& angle)
{
	angle = 3.14 * angle / 180;
}

/**
 * @brief 将单个点进行单应性变换
*/
Point2d Get_P_World(const Mat &H, Point2d P_pixel)
{
	double h11 = H.at<double>(0, 0); double h12 = H.at<double>(0, 1); double h13 = H.at<double>(0, 2);
	double h21 = H.at<double>(1, 0); double h22 = H.at<double>(1, 1); double h23 = H.at<double>(1, 2);
	double h31 = H.at<double>(2, 0); double h32 = H.at<double>(2, 1); double h33 = 1;

	double lamda = h31 * P_pixel.x + h32 * P_pixel.y + 1;
	Point2d P_World;
	P_World.x = (h11 * P_pixel.x + h12 * P_pixel.y + h13) / lamda;
	P_World.y = (h21 * P_pixel.x + h22 * P_pixel.y + h23) / lamda;

	return P_World;
}

/**
 * @brief 将一组点进行单应性变换
*/
vector<Point2d> Get_Pts_World(const Mat& H, vector<Point> Pts_pixel)
{
	double h11 = H.at<double>(0, 0); double h12 = H.at<double>(0, 1); double h13 = H.at<double>(0, 2);
	double h21 = H.at<double>(1, 0); double h22 = H.at<double>(1, 1); double h23 = H.at<double>(1, 2);
	double h31 = H.at<double>(2, 0); double h32 = H.at<double>(2, 1); double h33 = 1;
	vector<Point2d> Pts_world;
	for (auto P_pixel = Pts_pixel.begin(); P_pixel != Pts_pixel.end(); ++P_pixel)
	{
		double lamda = h31 * P_pixel->x + h32 * P_pixel->y + 1;
		Pts_world.push_back(Point2d((h11 * P_pixel->x + h12 * P_pixel->y + h13) / lamda,
																(h21 * P_pixel->x + h22 * P_pixel->y + h23) / lamda));
	}
	return Pts_world;
}

/**
 * @brief 由离散点拟合直线
*/
void Line::fitting(void)
{
	Vec4f cor;      //最小二乘法的计算结果
	fitLine(*pts, cor, DIST_L2, 0, 0.01, 0.01);
	k = cor[1] / cor[0];
	angle = atan(k);
	rad2degree(angle);
	if (angle > 0)	//Line从左上到右下(左偏)
		angle = 90 - angle;		//>0
	else						//Line从右上到左下(右偏)
		angle = -90 - angle;  //<0
	mid = Point(cor[2], cor[3]); //拟合出的直线的中点
}

/**
 * @brief 先将离散点变换到BEV坐标系下,再由最小二乘法计算结果
 * @param H 像素坐标系到BEV的单应矩阵
*/
void Line::fitting_BEV(const Mat &H)
{
	vector<Point2d> pts_midline = Get_Pts_World(H, *pts); //把中线上的所有点映射的BEV中
	Vec4f cor;      //最小二乘法的计算结果
	fitLine(pts_midline, cor, DIST_L2, 0, 0.01, 0.01);
	k = cor[1] / cor[0];
	angle = atan(k);
	rad2degree(angle);
	if (angle > 0)	//Line从左上到右下(左偏)
		angle = 90 - angle;		//>0
	else						//Line从右上到左下(右偏)
		angle = -90 - angle;  //<0
	mid = Get_P_World(H, Point2d(cor[2], cor[3])); //拟合出的直线的中点
}

/**
 * @brief 在图中画出拟合的直线
*/
void Line::draw(Mat& scrImage)
{
	if (this->pts->size() <= 0)
		return;
	this->fitting();
	double y1 = (*pts)[0].y;								 //第0个离散点
	double x1 = (y1 - mid.y) / k + mid.x;

	double y2 = (*pts)[(*pts).size() - 1].y; //最后一个离散点
	double x2 = (y2 - mid.y) / k + mid.x;

	line(scrImage, mid, Point(x1, y1), _color, 2);
	line(scrImage, mid, Point(x2, y2), _color, 2);

	circle(scrImage, mid, 2, Scalar(0, 0, 0), 3);
}