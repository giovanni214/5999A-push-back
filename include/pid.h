#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
  // PID gains
  double Kp;
  double Ki;
  double Kd;

  // Constructor
  PIDController(double p, double i, double d);

  //reset PID
  void reset();

  // Calculate control signal
  double calculateControlSignal(double setpoint, double measuredValue);

private:
  double previousError;
  double integral;
};

#endif // PIDCONTROLLER_H
