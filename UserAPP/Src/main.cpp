#include "predict.h"
#include "visual.h"
#include "yaml_config.h"
#include "iostream"
#include "opencv.hpp"
#include "line.h"
#include "control.h"
#include "task.h"

mutex mtx;

int main() 
{
	RobotController robot_a1;
	Custom custom(HIGHLEVEL);
	LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));
  loop_udpSend.start();
  loop_udpRecv.start();

	thread t_infer(Task_infer, ref(robot_a1), ref(custom));
	thread t_walk(Task_walk, ref(robot_a1), ref(custom));
	thread t_recv(Task_recv, ref(robot_a1), ref(custom));
	t_recv.join();
	t_infer.join();
	t_walk.join();
}


