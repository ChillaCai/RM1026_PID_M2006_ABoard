//
// Created by chill on 2024/10/26.
//

#include "../Inc/M2006_Motor.h"

extern uint8_t tx_data[8];

float linearMappingInt2Float(int in, int in_min, int in_max, float out_min, float out_max){
  if (in_min == in_max) return out_min;
  else return out_min + (out_max - out_min) / (in_max - in_min) * (in - in_min);
}

int linearMappingFloat2Int(float in, float in_min, float in_max, int out_min, int out_max){
  if (out_min == out_max) return out_min;
  else return (int)(out_min + (out_max - out_min) * (in - in_min) / (in_max - in_min));
}



M2006_Motor::M2006_Motor()
    : pid_single_vel(10.0, 0.0, 0.0, 0.0, 10),
      pid_cascade_vel(0.0065, 0.0, 0.0, 0, 10),
      pid_cascade_ang(3000.0, 1.0, 500.0, 10, 20000)
{
  ratio_ = 36.0f; 		// 电机减速比
  angle_ = 0.0f; 	        // deg 输出端累计转动角度
  delta_angle_ = 0.0f; 		// deg 输出端新转动的角度
  ecd_angle_ = 0.0f; 		// deg 当前电机编码器角度
  last_ecd_angle_ = 0.0f;	// deg 上次电机编码器角度
  delta_ecd_angle_ = 0.0f; 	// deg 编码器端新转动的角度
  rotate_speed_ = 0.0f;         // dps 反馈转子转速
  torque_ = 0.0f;               // 反馈转矩

  current_input_ = 0;           // 输入控制电流
  ref_vel_ = 540.0;             // 单环控制预期转速
  ref_ang_ = 20;                // 双环控制预期角度
}
void M2006_Motor::CanRxMsgCallback(uint8_t rx_data[8]){

  last_ecd_angle_ = ecd_angle_;

  int16_t ecd_angle = (int16_t)((uint16_t)rx_data[0] << 8) | rx_data[1];
  ecd_angle_ = linearMappingInt2Float(ecd_angle, 0, 8191, 0.0, 360.0);

  float delta = ecd_angle_ - last_ecd_angle_;
  // 编码器变化角度过零点修正
  if (delta < -180.0) delta_ecd_angle_ = delta + 360.0;
  else if (delta > 180.0) delta_ecd_angle_ = delta - 360.0;
  else delta_ecd_angle_ = delta;

  int16_t rotate_speed_rpm = (int16_t)((uint16_t)rx_data[2] << 8) | rx_data[3];
  rotate_speed_ = rotate_speed_rpm * 6.0;

  torque_ = (int16_t)((uint16_t)rx_data[4] << 8) | rx_data[5];

  delta_angle_ = delta_ecd_angle_ / ratio_;
  angle_ += delta_angle_;

//  PidInputSingle();
  PidInputCascade();
}

void M2006_Motor::PidInputSingle() {
  current_input_ = pid_single_vel.calc(ref_vel_, rotate_speed_);

  int current_tx = linearMappingFloat2Int(current_input_, -10.0, 10.0, -10000, 10000);
  tx_data[2] = current_tx >> 8;
  tx_data[3] = current_tx;
}

void M2006_Motor::PidInputCascade() {
  float ref_vel = pid_cascade_ang.calc(ref_ang_, angle_);

  current_input_ = pid_cascade_vel.calc(ref_vel, rotate_speed_);

  int current_tx = linearMappingFloat2Int(current_input_, -10.0, 10.0, -10000, 10000);
  tx_data[2] = current_tx >> 8;
  tx_data[3] = current_tx;
}