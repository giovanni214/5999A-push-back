#include "config.h"
#include "main.h"
#include "odometry.h"
#include "pid.h"
#include "pros/rtos.hpp"
#include <math.h>

static double shortestAngleDiff(double target, double current) {
  // returns difference target - current mapped to (-180, 180]
  double diff = fmod((target - current) + 540.0, 360.0) - 180.0;
  return diff;
}

void turnToAngle(PIDController &PID, double targetAngle, double tolerance = 1,
                 bool debugMode = true) {
  while (imu.is_calibrating()) {
    pros::delay(20);
  }

  int count = 0;
  while (std::fabs(targetAngle - globalAngle) > tolerance) {
    if (count > 3000)
      break;

    double control = PID.calculateControlSignal(targetAngle, globalAngle);
    if (control > 12000)
      control = 12000;
    if (control < -12000)
      control = -12000;

    left_mg.move_voltage(-control);
    right_mg.move_voltage(control);
    count += 10;
    pros::delay(10);
  }

  left_mg.move_voltage(0);
  right_mg.move_voltage(0);

  if (debugMode) {
    FILE *file_append = fopen("/usd/auto-turn-pid.txt", "a"); // append mode

    if (file_append != NULL) {
      // Write targetAngle, PID gains, tolerance, and globalAngle as the
      // "result"
      fprintf(file_append,
              "target: %.2f, Kp: %.2f, Ki: %.6f, Kd: %.2f, tol: %.2f, actual: "
              "%.2f error(%%): %.2f\n",
              targetAngle, PID.Kp, PID.Ki, PID.Kd, tolerance, globalAngle,
              (globalAngle - targetAngle) / targetAngle * 100);
      fclose(file_append);
    } else {
      printf("Failed to open file for appending\n");
    }
  }
}

void moveForward(PIDController &PID, double targetDistance,
                 double tolerance = 1, bool debugMode = true) {
  //TODO move robot forward x amount of inches
}

void turnAndMoveToPoint(double x, double y) {
  //TODO turn robot using turnToAngle, finding angle with atan2
  //TODO move robot x amount of inches. Use pythagorean theorem.
}

void resetIMU(PIDController &pid) {
  pros::delay(500);
  pid.reset();
  imu.set_rotation(0); // force it here
  pros::delay(100);    // let IMU update
}

void autonomous() {
  for (double d = 0; d < 20; d += 5) {
    for (double p = 10; p < 100; p += 5) {
      PIDController turningPID(p, 0.001, d);
      turnToAngle(turningPID, 360, 1, true);

      resetIMU(turningPID);
      turnToAngle(turningPID, 180, 1, true);

      resetIMU(turningPID);
      turnToAngle(turningPID, 90, 1, true);
      resetIMU(turningPID);
    }

    imu.reset(true);
  }
}