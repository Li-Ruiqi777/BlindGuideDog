#ifndef __CONTROL__H
#define __CONTROL__H
#include "pid.h"
#include "line.h"
#include "opencv.hpp"
#include "task.h"

using namespace cv;
class Custom;

class RobotController
{
public:
  typedef enum
  {
    Forward = 0,    // 正常寻迹模式
    Obstacle = 1,   // 开环直行模式
    Horizontal = 2, // 平移模式
    LoseLine = 3,   // 丢线模式
    Avoid = 4,      // 避障模式
    STOP = 5,       // 停止模式
  } Mode_t;         // 四足的运动模式

  typedef enum
  {
    Mid = 0,    // 在盲道上
    Left,       // 在盲道左边
    Right,      // 在盲道右边
  } Position_t; // 四足相对于盲道的位置

  typedef enum
  {
    Dir_Left = 0,  // 向左平移
    Dir_Right,     // 向右平移
  } Avoid_Dir; // 四足避障时的移动方向

  RobotController();
  ~RobotController() {}

  Line midLine, leftLine, rightLine;
  PID yaw_pid = PID(0, 0, 0);  // 输入为midline的夹角
  PID side_pid = PID(0, 0, 0); // 输入为midline的中点横坐标
  Mode_t mode;
  Position_t position;
  float forward_speed = 0; // 取值范围:-1~1,对应 -0.7~1 (m/s)
  float side_speed = 0;    // 取值范围:-1~1,对应 -0.4~0.4 (m/s). 大于0向左，小于0向右
  float yaw_speed = 0;     //(rad/s) 大于0是逆时针,小于0是顺时针

  // 避障相关
  double avg, max, min, depth; // 各种深度
  int stage_flag = 0;          // 用于避障模式
  float start_x = 0;           // 避障模式起始的X坐标
  float start_y = 0;           // 避障模式起始的Y坐标
  Avoid_Dir m_dir = Right;

  // 录制视频相关
  bool isVideo = false;

  // 锁定相关
  bool isLocking = false;

public:
  void calculate_speed(void);
  void mode_choose1(Mat &pred, Mat &H);

  // void Judge_Avoid(Mat &depth_image, Rect ROI, Custom &custom);
  // void mode_choose2(Mat &depth_image, Rect ROI, Custom &custom);

  void Judge_Avoid(uint16_t thresh, Avoid_Dir dir, Mat &depth_image, Rect ROI, Custom &custom);
  void mode_choose2(uint16_t thresh, Mat &depth_image, Rect ROI, Custom &custom);
};

#endif
