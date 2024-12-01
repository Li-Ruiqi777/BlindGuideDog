#include "control.h"
#include <vector>
#include <iostream>
#include <math.h>
#include "visual.h"

using namespace std;
using namespace cv;

RobotController::RobotController()
{
	mode = STOP;
	yaw_pid.set_target(0);
	yaw_pid.set_Limitval(1);
	yaw_pid.set_param(0, 0, 0);

	side_pid.set_target(320);
	side_pid.set_Limitval(0.5);
	side_pid.set_param(0, 0, 0);

	leftLine._color = Scalar(255, 0, 0);
	rightLine._color = Scalar(0, 255, 0);
	midLine._color = Scalar(0, 0, 255);

	avg = -1;
	min = -1;
	max = -1;
	depth = -1;
	this->stage_flag = 0;
}

/**
 * @brief 利用种子法确定3边的所有离散点
 *				利用中线上离散点转成BEV坐标系,再进行最小二乘法直线拟合,
					并执行机器人的状态机调度
 * @param pred 预测结果(单通道)
*/
void RobotController::mode_choose1(Mat &pred, Mat &H)
{
	/* initialize */
	const uint16_t row = pred.rows;
	const uint16_t col = pred.cols;
	const uint16_t start_row = row - 10;
	const uint16_t end_row = (row / 2) + 20;
	const uint8_t step = 20;
	const uint8_t max_samplePts = 1 + (row / 2 - 30) / step;
	const uint8_t bord_thresh = 3;			// Line为边界还是普通的阈值
	const uint8_t position_thresh = 40; // 判断四足相对于盲道的位置的阈值
	uint16_t x0 = col / 2;
	uint8_t Pts_Left = 0;	 // x=0的点的个数
	uint8_t Pts_Right = 0; // x=640的点的个数

	if (leftLine.pts != nullptr)
		delete leftLine.pts;
	if (rightLine.pts != nullptr)
		delete rightLine.pts;
	if (midLine.pts != nullptr)
		delete midLine.pts;

	leftLine.pts = new vector<Point>;
	rightLine.pts = new vector<Point>;
	midLine.pts = new vector<Point>;

	/* ---------------------- 1.确定所有离散点 ---------------------- */
	for (uint16_t idx = start_row; idx > end_row; idx -= step)
	{
		uint8_t *row_ptr = pred.ptr<uint8_t>(idx); // 指向当前行且x = 0的指针
		int x1 = 0, x2 = 0;

		// (1)种子初始点在盲道上
		if (row_ptr[x0] == 1)
		{
			/* 从中线向左边遍历，确定该行的左边界 */
			for (x1 = x0; x1 > 0; --x1)
			{
				if (row_ptr[x1] == 0)
					break;
			}
			if (x1 == 0)
				Pts_Left++;
			leftLine.pts->push_back(Point(x1, idx));

			/* 从中线向右边遍历，确定该行的右边界 */
			for (x2 = x0; x2 < col; ++x2)
			{
				if (row_ptr[x2] == 0)
					break;
			}
			if (x2 == col)
				Pts_Right++;
			rightLine.pts->push_back(Point(x2, idx));
		}

		// (2)种子初始点不在盲道上
		else if (row_ptr[x0] == 0)
		{
			// (2.1)最左边的点是背景点,从左往右扫描,先找左边界
			if (row_ptr[0] == 0)
			{
				/* 左边界确定 */
				for (x1 = 0; x1 < col; ++x1)
				{
					if (row_ptr[x1] == 1)
						break;
				}
				if (x1 == col) // 说明本行没有盲道元素,直接跳过
				{
					continue;
				}
				leftLine.pts->push_back(Point(x1, idx));

				/* 右边界确定 */
				for (x2 = x1; x2 < col; ++x2)
				{
					if (row_ptr[x2] == 0)
						break;
				}
				if (x2 == col)
					Pts_Right++;
				rightLine.pts->push_back(Point(x2, idx));
			}

			// (2.2)最右边的点是背景点,从右往左扫描,先找右边界
			else if (row_ptr[col - 1] == 0)
			{
				/* 右边界确定 */
				for (x2 = col - 1; x2 > 0; --x2)
				{
					if (row_ptr[x2] == 1)
						break;
				}
				if (x2 == 0) // 说明本行没有盲道元素,直接跳过
				{
					continue;
				}
				rightLine.pts->push_back(Point(x2, idx));

				/* 左边界确定 */
				for (x1 = x2; x1 > 0; --x1)
				{
					if (row_ptr[x1] == 0)
						break;
				}
				if (x1 == 0)
					Pts_Left++;
				leftLine.pts->push_back(Point(x1, idx));
			}
		}

		/* 中点确定 */
		x0 = (x1 + x2) / 2;
		midLine.pts->push_back(Point(x0, idx));
	}

	/* ---------------------- 2.机器人状态机调度 ---------------------- */
	uint8_t num_sample = midLine.pts->size(); // 实际采集到的离散点个数
	if (num_sample == 0)
	{
		mode = LoseLine;
	}

	else if (num_sample < max_samplePts - bord_thresh) // 11 - 3 = 8
	{
		mode = Obstacle;
	}

	else
	{
		(Pts_Left >= bord_thresh) ? (leftLine.sta = Line::Border) : (leftLine.sta = Line::Normal);
		(Pts_Right >= bord_thresh) ? (rightLine.sta = Line::Border) : (rightLine.sta = Line::Normal);
		midLine.fitting_BEV(H); // 先把中线的离散点转到BEV再进行直线拟合
		/* 判断四足相对于盲道的位置 */
		if (midLine.mid.x > 320 + position_thresh)
		{
			position = Left;
		}
		else if (midLine.mid.x < 320 - position_thresh)
		{
			position = Right;
		}
		else
		{
			position = Mid;
		}

		/* 判断运动模式 */
		if (leftLine.sta == Line::Normal && rightLine.sta == Line::Normal)
		{
			mode = Forward;
		}
		else
		{
			mode = Horizontal;
		}
	}
}

/**
 * @brief 由当前模式及相关信息计算速度
 */
void RobotController::calculate_speed(void)
{
	if (mode == Forward)
	{
		/* 前进速度 */
		forward_speed = 0.4f;
		/* 水平速度 */
		side_pid.set_param(0.006, 0, 0.003);
		side_pid.calculate_Error(midLine.mid.x);
		side_speed = side_pid.get_Output();
		/* 旋转速度 */
		yaw_pid.set_param(0.015, 0, 0.015);
		yaw_pid.calculate_Error(midLine.angle);
		yaw_speed = -yaw_pid.get_Output();
	}

	else if (mode == Horizontal)
	{
		/* 前进速度 */
		forward_speed = 0.3f;
		/* 水平速度 */
		side_pid.set_param(0.006, 0, 0.003);
		side_pid.calculate_Error(midLine.mid.x);
		side_speed = side_pid.get_Output();
		/* 旋转速度 */
		yaw_pid.set_param(0.015, 0, 0.015);
		yaw_pid.calculate_Error(midLine.angle);
		yaw_speed = -yaw_pid.get_Output();
	}

	else if (mode == Obstacle) // 开环直走
	{
		/* 前进速度 */
		forward_speed = 0.4f;
		/* 水平速度 */
		side_speed = 0;
		/* 旋转速度 */
		yaw_speed = 0;
	}

	else if (mode == LoseLine) // 自转直到找到线
	{
		/* 前进速度 */
		forward_speed = 0;
		/* 狗在盲道右边,向左寻找盲道 */
		if (position == Left)
		{
			yaw_speed = 1;
			side_speed = 0.1;
		}
		/* 狗在盲道左边,向右寻找盲道 */
		else if (position == Right)
		{
			yaw_speed = -1;
			side_speed = -0.1;
		}
		else
		{
			yaw_speed = 0;
			side_speed = 0;
		}
	}

	else if (mode == Avoid)
	{
		if (stage_flag == 1) // 平移
		{
			forward_speed = 0;
			yaw_speed = 0;
			switch (this->m_dir)
			{
				case Dir_Left:  side_speed = 0.2f;  break;
				case Dir_Right: side_speed = -0.2f; break;
				default: 				side_speed = -0.2f; break;
			}
		}
		else if (stage_flag == 2) // 直行
		{
			forward_speed = 0.2f;
			yaw_speed = 0;
			side_speed = 0;
		}
		else if (stage_flag == 3) // 平移
		{
			forward_speed = 0;
			yaw_speed = 0;
			switch (this->m_dir)
			{
				case Dir_Left:  side_speed = -0.2f; break;
				case Dir_Right: side_speed = 0.2f;  break;
				default: 				side_speed = 0.2f;  break;
			}
		}
		else
		{
			forward_speed = 0;
			yaw_speed = 0;
			side_speed = 0;
		}
	}

	else if (mode == STOP)
	{
		forward_speed = 0;
		yaw_speed = 0;
		side_speed = 0;
	}
}

// /**
//  * @brief 判断机器人是否进入避障模式
//  */
// void RobotController::Judge_Avoid(Mat &depth_image, Rect ROI, Custom &custom)
// {
// 	// 只有stage_flag = 0时才会进入Avoid模式
// 	if (this->mode == LoseLine || this->stage_flag!=0)
// 		return;
// 	/* 计算深度信息 */
// 	uint8_t step = 5;
// 	uint16_t thresh = 1500;// 600

// 	this->avg = Get_AverageDistance(depth_image, ROI, step);
// 	this->min = Get_MinDistance(depth_image, ROI, step);

// 	if ((0 < this->min) && (this->min < thresh))
// 	{
// 		this->mode = Avoid;		// 进入避障模式
// 		this->stage_flag = 1; // 进入平移阶段
// 		this->start_x = custom.state.position[0];
// 		this->start_y = custom.state.position[1];
// 		cout << "Enter Stage 1" << endl;
// 	}
// }

// /**
//  * @brief 避障模式中的状态机调度
//  */
// void RobotController::mode_choose2(Mat &depth_image, Rect ROI, Custom &custom)
// {
// 	uint16_t thresh = 1500; //600
// 	/* ---------------- 向左平移阶段 ---------------- */
// 	if (this->stage_flag == 1)
// 	{
// 		/* 计算深度信息 */
// 		uint8_t step = 5;
// 		this->avg = Get_AverageDistance(depth_image, ROI, step);
// 		this->min = Get_MinDistance(depth_image, ROI, step);
// 		// 1.当最小深度大于阈值时,说明平移的距离已经够了,进入stage2
// 		if (this->min > thresh || this->min == -1)
// 		{
// 			this->stage_flag = 2; // 进入直行阶段
// 		}
// 	}

// 	/* ---------------- 直行阶段 ---------------- */
// 	else if (this->stage_flag == 2)
// 	{
// 		float now_x = custom.state.position[0];
// 		// 2.当前进距离大于之前与障碍物间的距离时,说明直行距离已经够了,进入stage3,再次开始平移
// 		if ((now_x - this->start_x) > ((thresh*1.0)/1000)+0.5)// 单位为 m
// 		{
// 			// this->stage_flag = -1;
// 			// this->mode = STOP;
// 			this->stage_flag = 3;
// 		}
// 	}

// 	/* ---------------- 向右平移阶段 ---------------- */
// 	else if (this->stage_flag == 3)
// 	{
// 		float now_y = custom.state.position[1];
// 		// 3.当前的y和初始y非常接近或者当前位置已经在初始位置的右边了,说明回到了盲道上,此时退出避障模式
// 		if (abs(now_y - this->start_y) < 0.1 || now_y <= this->start_y)
// 		{
// 			this->stage_flag = 0;
// 			this->mode = Obstacle;

// 			// this->stage_flag = -1;
// 			// this->mode = STOP;
// 			cout << "Exit Avoid" << endl;
// 		}
// 	}
// }

void RobotController::Judge_Avoid(uint16_t thresh, Avoid_Dir dir, Mat &depth_image, Rect ROI, Custom &custom)
{
	// 只有stage_flag = 0时才会进入Avoid模式
	if (this->mode == LoseLine || this->stage_flag != 0)
		return;
	/* 计算深度信息 */
	uint8_t step = 5;

	this->avg = Get_AverageDistance(depth_image, ROI, step);
	this->min = Get_MinDistance(depth_image, ROI, step);

	if ((0 < this->min) && (this->min <= thresh))
	{
		this->mode = Avoid;		// 进入避障模式
		this->stage_flag = 1; // 进入平移阶段
		this->m_dir = dir;		// 避障移动方向
		this->start_x = custom.state.position[0];
		this->start_y = custom.state.position[1];
		cout << "Enter Stage 1" << endl;
	}
}

void RobotController::mode_choose2(uint16_t thresh, Mat &depth_image, Rect ROI, Custom &custom)
{
	/* ---------------- 向左平移阶段 ---------------- */
	if (this->stage_flag == 1)
	{
		/* 计算深度信息 */
		uint8_t step = 5;
		this->avg = Get_AverageDistance(depth_image, ROI, step);
		this->min = Get_MinDistance(depth_image, ROI, step);
		// 1.当最小深度大于阈值时,说明平移的距离已经够了,进入stage2
		if (this->min > thresh || this->min == -1)
		{
			this->stage_flag = 2; // 进入直行阶段
		}
	}

	/* ---------------- 直行阶段 ---------------- */
	else if (this->stage_flag == 2)
	{
		float now_x = custom.state.position[0];
		// 2.当前进距离大于之前与障碍物间的距离时,说明直行距离已经够了,进入stage3,再次开始平移
		if ((1000 * (now_x - this->start_x)) > (thresh + 200)) // 单位为 mm
		{
			// this->stage_flag = -1;
			// this->mode = STOP;
			this->stage_flag = 3;
		}
	}

	/* ---------------- 向右平移阶段 ---------------- */
	else if (this->stage_flag == 3)
	{
		float now_y = custom.state.position[1];
		// 一开始向左平移,则此时向右平移回到盲道上
		if (this->m_dir == Dir_Left)
		{
			// 3.当前的y和初始y非常接近或者当前位置已经在初始位置的右边了,说明回到了盲道上,此时退出避障模式
			if (abs(now_y - this->start_y) < 0.1 || now_y <= this->start_y)
			{
				this->stage_flag = 0;
				this->mode = Obstacle;
				cout << "Exit Avoid" << endl;
				// this->stage_flag = -1;
				// this->mode = STOP;
			}
		}

		// 一开始向右平移,则此时向左平移回到盲道上
		else
		{
			// 3.当前的y和初始y非常接近或者当前位置已经在初始位置的右边了,说明回到了盲道上,此时退出避障模式
			if (abs(this->start_y - now_y) < 0.1 || now_y >= this->start_y)
			{
				this->stage_flag = 0;
				this->mode = Obstacle;
				cout << "Exit Avoid" << endl;
			}
		}
	}

}