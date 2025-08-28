#include "config.h"
#include "main.h"
#include "odometry.h"
#include "pid.h"
#include <math.h>

void autonomous() {
  while (imu.is_calibrating()) {
    pros::delay(20);
  }

  PIDController turning(100, 0.001, 60);
  double targetAngle = 180;
  double tolerance = 1;

  while (std::fabs(targetAngle - globalAngle) > tolerance) {
    double control = turning.calculateControlSignal(targetAngle, globalAngle);
    if (control > 12000)
      control = 12000;
    if (control < -12000)
      control = -12000;

    left_mg.move_voltage(-control);
    right_mg.move_voltage(control);
    pros::delay(10);
  }

  left_mg.move_voltage(0);
  right_mg.move_voltage(0);
}