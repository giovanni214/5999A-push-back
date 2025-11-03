#include "config.h"
#include "main.h"
#include "odometry.h" // Provides globalX, globalY, globalAngle
#include "pid.h"
#include "pros/rtos.hpp"
#include <math.h> // Provides M_PI, atan2, sqrt, pow

// This constant is no longer needed for moveForward,
// as we are using odometry for distance.
// const double TICKS_PER_INCH = 71.62;

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

  while (std::fabs(shortestAngleDiff(targetAngle, globalAngle)) > tolerance) {
    if (count > 3000) // 3-second timeout
      break;

    // Calculate the error using shortestAngleDiff
    double error = shortestAngleDiff(targetAngle, globalAngle);

    // Feed this error to the PID. We want to drive the error to 0.
    // Setpoint = 0, Measured = -error
    double control = PID.calculateControlSignal(0, error);

    // Cap control signal
    if (control > 12000)
      control = 12000;
    if (control < -12000)
      control = -12000;

    // A positive control signal turns LEFT (CCW)
    left_mg.move_voltage(control);
    right_mg.move_voltage(-control);
    count += 10;
    pros::delay(10);
  }

  left_mg.brake();
  right_mg.brake();

  if (debugMode) {
    FILE *file_append = fopen("/usd/auto-turn-pid.txt", "a"); // append mode
    if (file_append != NULL) {
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

void moveForward(PIDController &movePID, double targetDistance,
                 double tolerance = 1, bool debugMode = true) {

  // 1. Set targets and store initial state
  double targetHeading = globalAngle; // Target heading is the current heading
  double startX = globalX;
  double startY = globalY;

  // 2. Reset PIDs
  movePID.reset();

  double currentDistance = 0;

  int count = 0;
  while (std::fabs(targetDistance - currentDistance) > tolerance) {
    if (count > 5000) // 10-second timeout
      break;

    // --- Distance PID Calculation (using odometry) ---
    // Calculate distance traveled from the start point
    currentDistance = std::sqrt(std::pow(globalX - startX, 2) +
                                std::pow(globalY - startY, 2));

    double error = targetDistance - currentDistance;
    double moveControl = movePID.calculateControlSignal(0, -error);

    // --- Cap signals ---
    if (moveControl > 12000)
      moveControl = 12000;
    if (moveControl < -12000)
      moveControl = -12000;

    // --- Apply voltage ---
    left_mg.move_voltage(-moveControl);
    right_mg.move_voltage(-moveControl);

    count += 10;
    pros::delay(10);
  }

  // Stop motors
  left_mg.brake();
  right_mg.brake();

  pros::delay(1000);

  if (debugMode) {
    FILE *file_append = fopen("/usd/new-auto-move-pid.txt", "a"); // append mode
    if (file_append != NULL) {
      fprintf(
          file_append,
          "target: %.2f in, Kp: %.2f, Ki: %.2f, Kd: %.2f, tol: %.2f, actual: "
          "%.2f in, time: %dms\n",
          targetDistance, movePID.Kp, movePID.Ki, movePID.Kd, tolerance,
          currentDistance, count);
      fclose(file_append);
    } else {
      printf("Failed to open file for appending\n");
    }
  }
}

void resetIMU() {
  pros::delay(1000);
  imu.set_rotation(0); // force it here
  pros::delay(100);    // let IMU update
}

void resetPID(PIDController &pid) { pid.reset(); }

void testTune() {
  PIDController turningPID(85, 0.001, 10); // set in stone for now

  for (int move_p = 200; move_p < 300; move_p += 10) {
    for (int move_d = 50; move_d < 100; move_d += 5) {
      PIDController movingPID(move_p, 0.001, move_d);

      resetIMU();
      resetPID(movingPID);
      resetPID(turningPID);
      moveForward(movingPID, 12, 1, true);

      resetIMU();
      resetPID(movingPID);
      resetPID(turningPID);
      moveForward(movingPID, 24, 1, true);

      resetIMU();
      resetPID(movingPID);
      resetPID(turningPID);
      moveForward(movingPID, 48, 1, true);

      resetIMU();
      resetPID(turningPID);
      turnToAngle(turningPID, 180);
    }
  }
}

double radiansToDegrees(double radians) { return radians * (180.0 / M_PI); }

// Returns the angle (in degrees) from (x1, y1) to (x2, y2)
double getAngle(double x1, double y1, double x2, double y2) {
  double radians = std::atan2(y2 - y1, x2 - x1);
  return 90 - radiansToDegrees(radians);
}

double getDistance(double x1, double y1, double x2, double y2) {
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

void turnAndMoveToPoint(PIDController &turnPID, PIDController &movePID,
                        double newX, double newY) {
  double turningAngle = getAngle(globalX, globalY, newX, newY);
  double moveDistance = getDistance(globalX, globalY, newX, newY);

  turnToAngle(turnPID, turningAngle, 1, false);
  moveForward(movePID, moveDistance, 0.5, false);
}

void autonomous() {
  // --- Example of how to use the new functions ---
  // You must define these PID controllers with gains you have tuned.
  while (imu.is_calibrating())
    pros::delay(50);

  globalX = 57.5;
  globalY = 30.75;
  PIDController movingPID(200, 1, 0);
  PIDController smallTurningPID(250, 1, 0);   // for small angles
  PIDController bigTurningPID(85, 0.001, 10); // for big angles

  turnAndMoveToPoint(smallTurningPID, movingPID, 48.25, 57.66);
  pros::delay(1000);

  double centerAngle = getAngle(globalX, globalY, 72, 72);

  PIDController smallTurn(120, 0.001, 10);
  turnToAngle(smallTurn, globalAngle + centerAngle);

  double moveDistance = getDistance(globalX, globalY, 61, 62);
  controller.set_text(0, 0, std::to_string(moveDistance));
  
  moveForward(movingPID, moveDistance);

  middle_motor.move(1 * 127);
  top_motor.move(1 * 127);
}