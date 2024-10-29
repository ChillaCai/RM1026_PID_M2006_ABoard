//
// Created by chill on 2024/10/26.
//

#ifndef CAN_PID_A_PID_H
#define CAN_PID_A_PID_H
#include <cstdint>

class PID {
public:
  PID(float kp, float ki, float kd, float i_max, float out_max);
  float calc(float ref, float fdb);
private:
  float kp_, ki_, kd_;
  float i_max_, out_max_;

  float output_;
  float ref_, fdb_;
  float err_, err_sum_, last_err_;
  float pout_, iout_, dout_;
};

#endif // CAN_PID_A_PID_H
