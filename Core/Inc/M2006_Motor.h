//
// Created by chill on 2024/10/26.
//

#ifndef CAN_PID_A_M2006_MOTOR_H
#define CAN_PID_A_M2006_MOTOR_H
#include <cstdint>
#include "PID.h"

class M2006_Motor {
private:
  float ratio_;                // 电机减速比
  float angle_;                // deg 输出端累计转动角度
  float delta_angle_;          // deg 输出端新转动的角度
  float ecd_angle_;            // deg 当前电机编码器角度
  float last_ecd_angle_;       // deg 上次电机编码器角度
  float delta_ecd_angle_;      // deg 编码器端新转动的角度
  float rotate_speed_;         // dps 反馈转子转速
  float torque_;               // 反馈转矩

  float current_input_;        // 输入的控制电流 float
  float ref_vel_;              // 单环控制预期速度
  float ref_ang_;              // 双环控制预期角度

  PID pid_single_vel;
  PID pid_cascade_vel;
  PID pid_cascade_ang;

public:
  M2006_Motor();
  void CanRxMsgCallback(uint8_t rx_data[8]);
  void PidInputSingle();
  void PidInputCascade();
};

#endif // CAN_PID_A_M2006_MOTOR_H
