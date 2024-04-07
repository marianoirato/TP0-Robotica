#include "PIDControl.h"

PIDControl::PIDControl(double kp, double ki, double kd, double Ts) :
  kp_(kp), ki_(ki), kd_(kd), Ts_(Ts), error_(0), integral_(0), derivative_(0), lastError_(0) {}

double PIDControl::calculate(double setPoint, double processVariable) {
  error_ = setPoint - processVariable;
  integral_ += error_ * Ts_;
  derivative_ = (error_ - lastError_) / Ts_;
  lastError_ = error_;
  return (kp_ * error_) + (ki_ * integral_) + (kd_ * derivative_);
}