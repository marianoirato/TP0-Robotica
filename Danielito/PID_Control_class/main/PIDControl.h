#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PIDControl {
public:
  PIDControl(double kp, double ki, double kd, double Ts);
  double calculate(double setPoint, double processVariable);

private:
  double kp_;
  double ki_;
  double kd_;
  double Ts_;
  double error_;
  double integral_;
  double derivative_;
  double lastError_;
};

#endif