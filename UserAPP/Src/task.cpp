#include "task.h"
static void rad2degree(float &angle)
{
  angle = angle * 180 / 3.14;
}

static void degree2rad(float &angle)
{
  angle = 3.14 * angle / 180;
}

void Custom::Init()
{
  cmd.mode = 0;
  cmd.gaitType = 0;
  cmd.speedLevel = 0;
  cmd.footRaiseHeight = 0;
  cmd.bodyHeight = 0;
  cmd.euler[0] = 0;
  cmd.euler[1] = 0;
  cmd.euler[2] = 0;
  cmd.velocity[0] = 0.0f;
  cmd.velocity[1] = 0.0f;
  cmd.yawSpeed = 0.0f;
}

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

void Custom::log()
{
  rad2degree(state.imu.rpy[0]);
  rad2degree(state.imu.rpy[1]);
  rad2degree(state.imu.rpy[2]);
  printf("roll=%.2f pitch= %.2f yaw=%.2f degree \r\n",
          state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]);
  // printf("height = %.2f \r\n", state.bodyHeight);
  printf("time = %.2fs\r\n", _second);
  // printf("mode = %d \r\n", state.mode);

}