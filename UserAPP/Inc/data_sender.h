#ifndef __DATA__H
#define __DATA__H
#include "stdint.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
class Data_Sender
{
public:
  Data_Sender();
  ~Data_Sender();
  float raw_data[9];
  void *Pack_Data(void);
  void Get_RawData(float yaw_speed, float side_speed, float forward_speed,
                   float angle_error, float side_error, float mode,
                   float avg, float stage, float min);
  bool Send(void);

public:
  uint8_t *send_array = nullptr;                           // 存待发送字节的动态数组
  uint8_t raw_data_len = 9;                                // 原始数据元素个数
  uint8_t send_Bytes = (2 + raw_data_len) * sizeof(float); // 一共需要多少字节

private:
  const uint32_t PORT = 7777;
  const char *IP_ADDR = "192.168.31.232"; //PC IP
  int sock_fd; // socket文件描述符
  struct sockaddr_in addr_serv;
};

#endif
