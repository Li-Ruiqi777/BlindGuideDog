#include "data_sender.h"
#include "iostream"
#include <cstring>
using namespace std;

/**
 * @brief 初始化Socket
 */
Data_Sender::Data_Sender()
{
  /* ----------准备Socket---------- */
  this->sock_fd = socket(AF_INET, SOCK_DGRAM, 0); // 建立udp socket
  memset(&addr_serv, 0, sizeof(addr_serv));
  this->addr_serv.sin_family = AF_INET;
  this->addr_serv.sin_addr.s_addr = inet_addr(this->IP_ADDR); // 设置address
  this->addr_serv.sin_port = htons(this->PORT);
  cout << "Socket Connected!" << endl;

  this->Get_RawData(0, 0, 0, 0, 0, 0x01, -1, -1, -1);
}

Data_Sender::~Data_Sender()
{
  close(this->sock_fd);
  cout << "Socket DisConnected!" << endl;
}

/**
 * @brief 将新数据填充到raw_data数组
 */
void Data_Sender::Get_RawData(float forward_speed, float side_speed, float yaw_speed,
                              float angle_error, float side_error, float mode,
                              float avg, float stage, float min)
{
  this->raw_data[0] = forward_speed;
  this->raw_data[1] = side_speed;
  this->raw_data[2] = yaw_speed;
  this->raw_data[3] = angle_error;
  this->raw_data[4] = side_error;
  this->raw_data[5] = mode;
  this->raw_data[6] = avg;
  this->raw_data[7] = stage;
  this->raw_data[8] = min;
}

/**
 * @brief 将raw_data数组中的数据打包成uint8_t字节型数组,存在this->send_array这个动态数组内
 */
void *Data_Sender::Pack_Data(void)
{
  if (this->send_array == nullptr)
    delete[] this->send_array;

  float *packed_data = new float[(2 + raw_data_len)]; // 先把raw_data和帧头帧尾打包成float的数组
  // 数据加上帧头帧尾
  packed_data[0] = 0xb3; // start
  memcpy(packed_data + 1, &raw_data, sizeof(raw_data));
  packed_data[raw_data_len + 1] = 0xb4; // end

  // 将数组转成二进制
  this->send_array = new uint8_t[this->send_Bytes];
  memcpy(send_array, packed_data, this->send_Bytes);
  delete[] packed_data;

  // cout << "packed_data" << 2 + raw_data_len << "个float" << endl;
  // cout << this->send_Bytes << endl;
  return send_array;
}

bool Data_Sender::Send(void)
{
  int send_num = 0;
  send_num = sendto(this->sock_fd, this->send_array, this->send_Bytes,
                    0, (struct sockaddr *)&(this->addr_serv), sizeof(this->addr_serv));
  //cout<<"send_num = "<<send_num<<endl;
  if (send_num < 0)
  {
    cout << "Send Error" << endl;
    return false;
  }
  else
    return true;
}