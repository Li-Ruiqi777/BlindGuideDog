#include "pid.h"

PID::PID(double kp, double ki, double kd)
{
  _kp = kp;
  _kd = kd;
  _ki = ki;
  _setpoint = 0;
  _Now_err = 0;
  _Sum_err = 0;
  _Diff_err = 0;
  _Sum_MAX = 9999999;
  _limit_val = 999999;
}

/**
 * @brief 设定参数
 */
void PID::set_param(double kp, double ki, double kd)
{
  _kp = kp;
  _kd = kd;
  _ki = ki;
}

/**
 * @brief 设定目标值
 */
void PID::set_target(double input)
{
  _setpoint = input;
}

/**
 * @brief 对输出限幅
 * @param output 
 * @param limit 必须是正值
*/
void PID::output_limit(double& output, double limit)
{
  if (output >= 0)
  {
    if (output >= limit)
      output = limit;
  }
  else if (output < 0)
  {
    if (-output >= limit)
      output = -limit;
  }
}

/**
 * @brief 根据公式计算输出(增量式)
 */
double PID::get_Output(void)
{
  double output = _kp * _Now_err + _ki * _Sum_err + _kd * _Diff_err;
  output_limit(output, _limit_val);
  return output;
}

/**
 * @brief 根据输入计算误差并更新参数
 * @param input
 */
void PID::calculate_Error(double input)
{
  double err = _setpoint - input;
  this->set_NewError(err);
}

/**
 * @brief 根据输入的误差更新参数
 * @param input
 */
void PID::set_NewError(double err)
{
  _Last_err = _Now_err;
  _Now_err = err;
  _Sum_err += _Now_err;
  _Diff_err = _Now_err - _Last_err;
  if (_Sum_err >= _Sum_MAX)
    _Sum_err = _Sum_MAX;
}

/**
 * @brief 清空所有误差
 */
void PID::empty_Error(void)
{
  _Last_err = 0;
  _Now_err = 0;
  _Sum_err = 0;
  _Diff_err = 0;
}

/**
 * @brief 设置输出幅值
 * @param limit 必须>0
*/
void PID::set_Limitval(double limit)
{
  _limit_val = limit;
}