#include "task.h"
#include "stdint.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

using namespace std;

void Task_recv(RobotController &robot_a1, Custom &custom)
{
  const uint32_t PORT = 7778;
  const char *IP_ADDR = "192.168.31.116"; //狗的IP地址
  int sock_fd; // socket文件描述符
  struct sockaddr_in addr_serv;
  struct sockaddr_in addr_client;
  int len = sizeof(addr_serv);
  /* ----------准备Socket---------- */
  sock_fd = socket(AF_INET, SOCK_DGRAM, 0); // 建立udp socket
  memset(&addr_serv, 0, sizeof(addr_serv));
  addr_serv.sin_family = AF_INET;
  // addr_serv.sin_addr.s_addr = htonl(INADDR_ANY); // 设置address
  addr_serv.sin_addr.s_addr = inet_addr(IP_ADDR);
  addr_serv.sin_port = htons(PORT);
  // 设置端口复用
  int opt = 1;
  setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  /* 绑定socket */
  if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
  {
    perror("bind error:");
    exit(1);
  }
  cout << "Video Socket Connected!" << endl;

  uint8_t recv_buf[20];



  while (1)
  {
    int recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf),
                            0, (struct sockaddr *)&(addr_client), (socklen_t *)&len);
    if (recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }

    if (recv_buf[0] == 0XAA)
    {
      cout << "开始录制视频" << endl;
      robot_a1.isVideo = true;
    }
    else if (recv_buf[0] == 0XBB)
    {
      cout << "结束录制视频" << endl;
      robot_a1.isVideo = false;
    }

    else if (recv_buf[0] == 0X44)
    {
      cout << "机器人被锁定" << endl;
      robot_a1.isLocking = true;
    }

    else if (recv_buf[0] == 0X55)
    {
      cout << "机器人被解锁" << endl;
      robot_a1.isLocking = false;
    }
    // recv_buf[recv_num] = '\0';
    // printf("client receive %d bytes: %s\n", recv_num, recv_buf);
  }
}
