#include "visual.h"
#include <iostream>
using namespace cv;
using namespace std;

/**
 * @brief 对输入的预处理,主要包括bgr2rgb、resize、归一化和Normalize
 * @param src 应该是摄像头拍摄的原始图像
 * @param yaml_config
 * @return 处理后的Mat
*/
Mat Image_Preprocess(cv::Mat& src, const YamlConfig& yaml_config)
{
	cv::Mat img;
	src.copyTo(img);
	cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
	if (yaml_config.is_resize)
	{
		cv::resize(img, img, cv::Size(yaml_config.resize_width, yaml_config.resize_height));
	}
	if (yaml_config.is_normalize)
	{
		img.convertTo(img, CV_32F, 1.0 / 255, 0);
		img = (img - 0.5) / 0.5;
	}
	return img;
}

/**
 * @brief 将以hwc排列的Mat中的信息复制到以chw排列的容器中
 * @param hwc_img
 * @param data 容器数据的首地址
*/
void hwc_img_2_chw_data(const cv::Mat& hwc_img, float* data)
{
	int rows = hwc_img.rows;
	int cols = hwc_img.cols;
	int chs = hwc_img.channels();
	for (int i = 0; i < chs; ++i)
	{
		cv::extractChannel(hwc_img, cv::Mat(rows, cols, CV_32FC1, data + i * rows * cols), i);
	}
}

/**
 * @brief 获得将预测图和原图进行融合可视化结果
*/
Mat addedImage(const cv::Mat& src, const cv::Mat& pred)
{
	Mat added, temp;
	equalizeHist(pred, temp);
	applyColorMap(temp, temp, 14);
	addWeighted(src, 0.7, temp, 0.3, 0, added);
	return added;
}
 
/**
 * @brief 使用种子法由图像底部->中线 得到 边界点 以及 中线 的坐标
*/
void Seed_method(Mat& pred, vector<Point>& Left, vector<Point>& Right, vector<Point>& Mid)
{
	/* initialize */
	const uint16_t row = pred.rows;
	const uint16_t col = pred.cols;
	const uint16_t start_row = row - 10;
	const uint16_t end_row = (row / 2) + 20;
	const uint8_t step = 20;

	uint16_t x0 = col / 2;

	for (uint16_t idx = start_row; idx > end_row; idx -= step)
	{
		uint8_t* row_ptr = pred.ptr<uint8_t>(idx); //指向当前行且x = 0的指针
		int x1 = 0, x2 = 0;

		// 1.种子初始点在盲道上
		if (row_ptr[x0] != 0)
		{
			/* 从中线向左边遍历，确定该行的左边界 */
			for (x1 = x0; x1 > 0; --x1)
			{
				if (row_ptr[x1] == 0)
					break;
			}
			Left.push_back(Point(x1, idx));

			/* 从中线向右边遍历，确定该行的右边界 */
			for (x2 = x0; x2 < col; ++x2)
			{
				if (row_ptr[x2] == 0)
					break;
			}
			Right.push_back(Point(x2, idx));
		}

		// 2.种子初始点不在盲道上
		else
		{
			if (row_ptr[0] == 0)// 狗左偏,从左往右扫描边界
			{
				/* 左边界确定 */
				for (x1 = 0; x1 < col; ++x1)
				{
					if (row_ptr[x1] == 1)
						break;
				}
				Left.push_back(Point(x1, idx));

				/* 右边界确定 */
				for (x2 = x1; x2 < col; ++x2)
				{
					if (row_ptr[x2] == 0)
						break;
				}
				Right.push_back(Point(x2, idx));
			}

			else// 狗右偏,从右往左扫描边界
			{
				/* 右边界确定 */
				for (x2 = col - 1; x2 > 0; --x2)
				{
					if (row_ptr[x2] == 1)
						break;
				}
				Right.push_back(Point(x2, idx));

				/* 左边界确定 */
				for (x1 = x2; x1 > 0; --x1)
				{
					if (row_ptr[x1] == 0)
						break;
				}
				Left.push_back(Point(x1, idx));
			}
		}

		/* 中点确定 */
		x0 = (x1 + x2) / 2;
		Mid.push_back(Point(x0, idx));

	}
}

/**
 * @brief 画出相关信息
*/
void darw_info(Mat& scrImage, vector<Point>& Left, vector<Point>& Right, vector<Point>& Mid)
{
	for (int i = 0; i < Mid.size(); ++i)
	{
		circle(scrImage, Mid[i], 1, Scalar(0, 0, 255));
		circle(scrImage, Left[i], 1, Scalar(255, 0, 0));
		circle(scrImage, Right[i], 1, Scalar(0, 255, 0));
	}
	line(scrImage, Point(scrImage.cols / 2, 0), Point(scrImage.cols / 2, scrImage.rows),
		Scalar(255, 255, 255), 4);//竖中线
	line(scrImage, Point(0, scrImage.rows / 2), Point(scrImage.cols, scrImage.rows / 2),
		Scalar(255, 255, 255), 4);//横中线
}


void darw_info(Mat& scrImage)
{
	line(scrImage, Point(scrImage.cols / 2, 0), Point(scrImage.cols / 2, scrImage.rows),
		Scalar(255, 255, 255), 4);//竖中线
	line(scrImage, Point(0, scrImage.rows / 2), Point(scrImage.cols, scrImage.rows / 2),
		Scalar(255, 255, 255), 4);//横中线
}

/**
 * @brief 计算逆透视变换成鸟瞰图的单应矩阵
 */
Mat calibration(void)
{
	const uint16_t height = 480, width = 640;
	vector<Point2f> points_pixel{ Point2f(311,254),Point2f(352,254),
							Point2f(239,417),Point2f(438,417) };//pix
	vector<Point2f> points_world{ Point2f(width / 2 - 40,0),Point2f(width / 2 + 40,0),
							Point2f(width / 2 - 40,height - 50),Point2f(width / 2 + 40,height - 50) };//pix
	Mat H = findHomography(points_pixel, points_world);
	return H;
}

/**
 * @brief 计算深度图ROI内的深度最大值
 * @return
 */
double Get_MaxDistance(Mat& depth_img, Rect& ROI, uint8_t step)
{
	double max_depth = 0;
	Mat img_roi(depth_img, ROI);
	for (int i = 0; i < ROI.height; ++i)
	{
		uint16_t* ptr = img_roi.ptr<uint16_t>(i);
		for (int j = 0; j < ROI.width; j += step)
		{
			if (ptr[j] > max_depth)
				max_depth = ptr[j];
		}
	}
	return max_depth;
}

/**
 * @brief 计算深度图ROI内的有效深度的最小值
 * @return <=0：无效深度;  >0 返回的是ROI内深度最小值,即距离最近的点
 */
double Get_MinDistance(Mat& depth_img, Rect& ROI, uint8_t step)
{
	double min_depth = 65536;
	double zero_num = 0;
	Mat img_roi(depth_img, ROI);
	for (int i = 0; i < ROI.height; ++i)
	{
		uint16_t* ptr = img_roi.ptr<uint16_t>(i);
		for (int j = 0; j < ROI.width; j += step)
		{
			if (min_depth > ptr[j] && ptr[j] != 0)
				min_depth = ptr[j];
			if (ptr[j] == 0)
				zero_num++;
		}
	}
	if (zero_num > (ROI.area() / 2 / step))
		return -1;

	else
		return min_depth;
}

/**
 * @brief 计算深度图ROI内的深度均值
 * @return -1:深度为0的点太多了; >0 返回的是ROI内深度的均值
 */
double Get_AverageDistance(Mat& depth_img, Rect& ROI, uint8_t step)
{
	double sum = 0;
	double zero_num = 0;
	Mat img_roi(depth_img, ROI);
	for (int i = 0; i < ROI.height; ++i)
	{
		uint16_t* ptr = img_roi.ptr<uint16_t>(i);
		for (int j = 0; j < ROI.width; j += step)
		{
			if (ptr[j] != 0)
				sum += ptr[j];
			else
				zero_num++;
		}
	}
	if (zero_num > (ROI.area() / 2 / step)) //超过一半都为0
		return -1;

	else
		return sum / ((ROI.area() / step) - zero_num);
}