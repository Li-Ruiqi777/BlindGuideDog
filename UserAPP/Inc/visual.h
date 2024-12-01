#ifndef __VISUAL__H
#define __VISUAL__H
#include "opencv.hpp"
#include "yaml_config.h"
#include <vector>
using namespace std;
using namespace cv;
/**
 * @brief 对输入的预处理,主要包括bgr2rgb、resize、归一化和Normalize
 * @param src 应该是摄像头拍摄的原始图像
 * @param yaml_config
 * @return 处理后的Mat
*/
cv::Mat Image_Preprocess(cv::Mat& src, const YamlConfig& yaml_config);

/**
 * @brief 将以hwc排列的Mat中的信息复制到以chw排列的容器中
 * @param hwc_img
 * @param data 容器数据的首地址
*/
void hwc_img_2_chw_data(const cv::Mat& hwc_img, float* data);

/**
 * @brief 获得将预测图和原图进行融合可视化结果
*/
cv::Mat addedImage(const cv::Mat& src, const cv::Mat& pred);

/**
 * @brief 使用种子法由横中线向远端得到 边界点 以及 中线 的坐标
*/
void Seed_method(Mat& pred, vector<Point>& Left, vector<Point>& Right, vector<Point>& Mid);

/**
 * @brief 画出相关信息
*/
void darw_info(Mat& scrImage, vector<Point>& Left, vector<Point>& Right, vector<Point>& Mid);
void darw_info(Mat& scrImage);

void darw_line(Mat& scrImage, vector<Point>& Left, vector<Point>& Right, vector<Point>& Mid);

Mat calibration(void);

double Get_MaxDistance(Mat &depth_img, Rect &ROI, uint8_t step);
double Get_MinDistance(Mat &depth_img, Rect &ROI, uint8_t step);
double Get_AverageDistance(Mat &depth_img, Rect &ROI, uint8_t step);

#endif
