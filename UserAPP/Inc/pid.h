#ifndef __PID__H
#define __PID__H
class PID
{
public:

  PID() = default;
  PID(double kp, double ki, double kd);
  ~PID() {}
  void set_param(double kp, double ki, double kd);
  void set_target(double input);                    // 设定目标值
  double get_Output(void);                          // 计算输出
  void output_limit(double& output, double limit);  //输出限幅
  void calculate_Error(double input);               // 由输入计算Error
  void set_NewError(double err);                    // 直接传入Error
  void empty_Error(void);                           // 清除累计误差
  void set_Limitval(double limit);                  // 设置输出限幅

//private:
  double _limit_val;
  double _kp, _ki, _kd;
  double _setpoint; // 目标设定值
  double _Now_err;  // 定义当前误差
  double _Sum_err;  // 总误差
  double _Diff_err; // 微误差
  double _Last_err; // 上一次误差
  double _Sum_MAX;  // 积分限幅
};

#endif