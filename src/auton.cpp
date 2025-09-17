#include "config.h"
#include "main.h"
#include "odometry.h"
#include "pid.h"
#include "pros/rtos.hpp"
#include <math.h>

void turnToAngle(PIDController &PID, double targetAngle, double tolerance = 1,
                 bool debugMode = true) {
  while (imu.is_calibrating()) {
    pros::delay(20);
  }

  while (std::fabs(targetAngle - globalAngle) > tolerance) {
    double control = PID.calculateControlSignal(targetAngle, globalAngle);
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

  if (debugMode) {
    FILE *file_append = fopen("/usd/turning-pid.txt", "a"); // append mode

    if (file_append != NULL) {
      // Write targetAngle, PID gains, tolerance, and globalAngle as the
      // "result"
      fprintf(file_append,
              "target: %.2f, Kp: %.2f, Ki: %.6f, Kd: %.2f, tol: %.2f, actual: "
              "%.2f\n",
              targetAngle, PID.Kp, PID.Ki, PID.Kd, tolerance, globalAngle);
      fclose(file_append);
    } else {
      printf("Failed to open file for appending\n");
    }
  }
}

void resetIMU(PIDController &pid) {
  pid.reset();
  resetAngle = true;
}

void autonomous() {
  PIDController turningPID(50, 0.001, 1);
  turnToAngle(turningPID, 360, 1, false);

  resetIMU(turningPID);
  // turnToAngle(turningPID, 180, 1, false);

  // resetIMU(turningPID);
  // turnToAngle(turningPID, 90, 1, true);

  // resetIMU(turningPID);
  // turnToAngle(turningPID, 45, 1, true);

  // resetIMU(turningPID);
  // turnToAngle(turningPID, 15, 1, true);

  // resetIMU(turningPID);
  // turnToAngle(turningPID, 5, 1, true);
}