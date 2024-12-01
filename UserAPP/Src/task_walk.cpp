#include "predict.h"
#include "visual.h"
#include "yaml_config.h"
#include "opencv.hpp"
#include <librealsense2/rs.hpp>
#include "task.h"
#include "data_sender.h"

void Stop_Init(Custom &custom)
{
  /* 静止站立 */
  custom.cmd.mode = 1;
  custom.cmd.gaitType = 0;
  custom.cmd.speedLevel = 0;
  custom.cmd.footRaiseHeight = 0;
  custom.cmd.bodyHeight = 0;
  custom.cmd.euler[0] = 0;
  custom.cmd.euler[1] = 0;
  custom.cmd.euler[2] = 0;
  custom.cmd.velocity[0] = 0.0f;
  custom.cmd.velocity[1] = 0.0f;
  custom.cmd.yawSpeed = 0.0f;
}

void Move_Init(Custom &custom)
{
  custom.cmd.mode = 2;
  custom.cmd.gaitType = 1; // 之前是2
  custom.cmd.velocity[0] = 0;
  custom.cmd.velocity[1] = 0;
  custom.cmd.yawSpeed = 0;
  custom.cmd.footRaiseHeight = 0.1;
}

void Task_walk(RobotController &robot_a1, Custom &custom)
{
  custom.udp.SetSend(custom.cmd);
  Move_Init(custom);
  /* 初始化下位机Logger */
  Data_Sender sender; // IP地址记得修改

  while (1)
  {
    custom.motiontime += custom.dt_ms; // ms
    custom._second += custom.dt;       // s

    custom.udp.GetRecv(custom.state);
    mtx.lock();
    if (robot_a1.isLocking)
    {
      custom.cmd.velocity[0] = 0;
      custom.cmd.velocity[1] = 0;
      custom.cmd.yawSpeed = 0;
    }
    else
    {
      custom.cmd.velocity[0] = robot_a1.forward_speed;
      custom.cmd.velocity[1] = robot_a1.side_speed;
      custom.cmd.yawSpeed = robot_a1.yaw_speed;
    }
    mtx.unlock();

    custom.udp.SetSend(custom.cmd);

    /*----------------- 终端输出 ------------------*/
    if (custom.motiontime % 1000 == 0)
    {
      cout << "-------------------------------" << endl;
      custom.log(); // UDP接收到的一些运行信息
      string dog_mode;
      switch (robot_a1.mode)
      {
      case 0:
        dog_mode = string("Forward");break;
      case 1:
        dog_mode = string("Obstacle");break;
      case 2:
        dog_mode = string("Horizontal");break;
      case 3:
        dog_mode = string("LoseLine");break;
      case 4:
        dog_mode = string("Avoid");break;
      case 5:
        dog_mode = string("STOP");break;
      default:break;
      }

      // cout << "forward = " << robot_a1.forward_speed << endl;
      // cout << "side    = " << robot_a1.side_speed << endl;
      // cout << "yaw     = " << robot_a1.yaw_speed << endl;
      // cout << "pos1 :  " << custom.state.position[0]<<endl;
      // cout << "pos2 :  " << custom.state.position[1]<<endl;
      // cout << "pos3 :  " << custom.state.position[2]<<endl;

      cout << "FSM = " << dog_mode << endl;
      cout << "-------------------------------" << endl
           << endl;
    }

    /*----------------- 上位机 5HZ ------------------*/
    if (custom.motiontime % 200 == 0)
    {
      sender.Get_RawData(robot_a1.forward_speed, robot_a1.side_speed, robot_a1.yaw_speed,
                         robot_a1.yaw_pid._Now_err, robot_a1.side_pid._Now_err, float(robot_a1.mode),
                         robot_a1.avg, float(robot_a1.stage_flag), robot_a1.min);
      sender.Pack_Data();
      sender.Send();
      // cout << "min = " << robot_a1.min << endl;
      // cout << "pos_X :  " << custom.state.position[0] << endl;
      // cout << "pos_Y :  " << custom.state.position[1] << endl;
    }

    this_thread::sleep_for(chrono::milliseconds(custom.dt_ms));
  }
}
