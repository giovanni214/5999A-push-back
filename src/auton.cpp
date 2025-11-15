#include "config.h"
#include "liblvgl/misc/lv_text.h"
#include "main.h"
#include "motors_loop.h"
#include "odometry.h" // Provides globalX, globalY, globalAngle
#include "pid.h"
#include "pros/rtos.hpp"
#include <math.h> // Provides M_PI, atan2, sqrt, pow
#include <string>

pros::c::optical_rgb_s_t hsv_to_rgb(double h, double s, double v) {
  pros::c::optical_rgb_s_t rgb;

  double c = v * s;
  double x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
  double m = v - c;

  double r, g, b;

  if (h < 60) {
    r = c, g = x, b = 0;
  } else if (h < 120) {
    r = x, g = c, b = 0;
  } else if (h < 180) {
    r = 0, g = c, b = x;
  } else if (h < 240) {
    r = 0, g = x, b = c;
  } else if (h < 300) {
    r = x, g = 0, b = c;
  } else {
    r = c, g = 0, b = x;
  }

  // --- FIX IS HERE ---
  // Return the 0.0-1.0 range, not the 0-255 range.
  rgb.red = (r + m);
  rgb.green = (g + m);
  rgb.blue = (b + m);
  // -------------------

  return rgb;
}

std::string getColor() {
  double hue = optical_sensor.get_hue();

  // Hue ranges:
  // Red ~ 0–20 or 340–360
  // Blue ~ 200–250
  if ((hue >= 0 && hue <= 20) || (hue >= 340 && hue <= 360)) {
    return "RED";
  }
  if (hue >= 200 && hue <= 250) {
    return "BLUE";
  }

  return "UNKNOWN";
}

static double shortestAngleDiff(double target, double current) {
  // returns difference target - current mapped to (-180, 180]
  double diff = fmod((target - current) + 540.0, 360.0) - 180.0;
  return diff;
}

void turnToAngle(PIDController &PID, double targetAngle, double tolerance = 1,
                 bool debugMode = true, int timeout = 3000) {
  while (imu.is_calibrating()) {
    pros::delay(20);
  }

  PID.reset();

  int count = 0;

  while (std::fabs(shortestAngleDiff(targetAngle, globalAngle)) > tolerance) {
    if (count > timeout) // 3-second timeout
      break;

    // Calculate the error using shortestAngleDiff
    double error = shortestAngleDiff(targetAngle, globalAngle);

    // Feed this error to the PID. We want to drive the error to 0.
    // Setpoint = 0, Measured = -error
    double control = PID.calculateControlSignal(0, -error);

    // Cap control signal
    if (control > 12000)
      control = 12000;
    if (control < -12000)
      control = -12000;

    // A positive control signal turns LEFT (CCW)
    left_mg.move_voltage(-control);
    right_mg.move_voltage(control);
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
                 double tolerance = 1, bool debugMode = true,
                 int timeout = 5000) {

  // 1. Set targets and store initial state
  double targetHeading = globalAngle; // Target heading is the current heading
  double startX = globalX;
  double startY = globalY;

  // 2. Reset PIDs
  movePID.reset();

  double currentDistance = 0;

  int count = 0;
  while (std::fabs(targetDistance - currentDistance) > tolerance) {
    if (count > timeout) // 10-second timeout
      break;

    // --- Distance PID Calculation (using odometry) ---
    double dx = globalX - startX;
    double dy = globalY - startY;
    double travelDir = targetHeading * M_PI / 180.0; // radians
    currentDistance = dy * std::cos(travelDir) + dx * std::sin(travelDir);

    double moveControl =
        movePID.calculateControlSignal(targetDistance, currentDistance);

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
                        double newX, double newY, int maxTurnTime = 3000,
                        int maxMoveTime = 5000) {
  double turningAngle = getAngle(globalX, globalY, newX, newY);
  double moveDistance = getDistance(globalX, globalY, newX, newY);

  turnToAngle(turnPID, turningAngle, 1, false, maxTurnTime);
  moveForward(movePID, moveDistance, 0.5, false, maxMoveTime);
}

void autonomous() {
  // --- Example of how to use the new functions ---
  // You must define these PID controllers with gains you have tuned.
  while (imu.is_calibrating())
    pros::delay(50);

  globalX = 60;
  globalY = 25;
  PIDController movingPID(200, 1, 0);
  PIDController smallTurningPID(120, 1, 20); // for small angles
  PIDController bigTurningPID(85, 0.01, 10); // for big angles

  intakeDir = 1;
  mode = 1;
  turnAndMoveToPoint(smallTurningPID, movingPID, 48, 48, 1000, 4000);

  double centerAngle = getAngle(globalX, globalY, 72, 71);

  // PIDController smallTurn(120, 0.001, 10);
  turnToAngle(bigTurningPID, centerAngle, 1, false, 1000);

  double moveDistance = getDistance(globalX, globalY, 62, 61);
  moveForward(movingPID, 18.5, 0.5, false, 1000);

  mode = 2; // output to middle
  gate_pneumatic.retract();
  pros::delay(50);

  int giveUpTime = 2000;
  int timeTaken = 0;
  int pnuematicCount = 0;

  while (optical_sensor.get_proximity() < 50) {
    if (timeTaken >= giveUpTime)
      break;

    if (pnuematicCount % 10 == 0) {
      punch_pneumatic.toggle();
    }

    timeTaken += 20;
    pnuematicCount++;
    pros::delay(20);
  }

  gate_pneumatic.extend();
  controller.set_text(0, 0, getColor());
  pros::delay(1500);

  PIDController longDistance(100, 0.01, 10);
  moveDistance = getDistance(globalX, globalY, 24, 24);
  controller.set_text(0, 0, std::to_string(moveDistance));
  moveForward(longDistance, -moveDistance, 1, false, 4000);
  pros::delay(50000);

  bigTurningPID.Kp += 15;
  turnToAngle(bigTurningPID, 0, 1, false, 1000);
  moveForward(movingPID, 18, 1, false, 1500);

  gate_pneumatic.retract();
  mode = 3;
}