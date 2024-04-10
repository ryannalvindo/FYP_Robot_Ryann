#include "PID.h"

PID::PID(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _integral = 0;
  _lastError = 0;
}

void PID::setConstants(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

float PID::compute(float target, float current) {
  float error = target - current;
  _integral += error;
  float derivative = error - _lastError;
  _lastError = error;
  
  return (_kp * error) + (_ki * _integral) + (_kd * derivative);
}

void PID::reset() {
  _integral = 0;
  _lastError = 0;
}
