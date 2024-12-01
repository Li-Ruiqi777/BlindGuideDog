#ifndef __TASK_H
#define __TASK_H

#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <mutex>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "control.h"

using namespace UNITREE_LEGGED_SDK;
using namespace std;
using namespace cv;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::A1),
                          udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
  {
    udp.InitCmdData(cmd);
    this->Init();
  }
  void UDPRecv();
  void UDPSend();
  void Init();
  void log();

  Safety safe;
  UDP udp;
  HighCmd cmd = {0};
  HighState state = {0};
  int motiontime = 0; //(unit:ms)
  float dt = 0.005;   // 0.001~0.01 (unit:s)
  int dt_ms = 1000 * dt;
  float _second = 0; //(unit:s)
};

class RobotController;
void Task_infer(RobotController &robot_a1, Custom &custom);
void Task_walk(RobotController &robot_a1, Custom &custom);
void Task_recv(RobotController &robot_a1, Custom &custom);
extern mutex mtx;
#endif
