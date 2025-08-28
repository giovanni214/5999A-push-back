#pragma once

class PIDController {
private:
  double Kp, Ki, Kd;
  double previousError, integral;

public:
  PIDController(double p, double i, double d);

  double calculateControlSignal(double setpoint, double measuredValue);
};
