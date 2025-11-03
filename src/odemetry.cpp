#include "config.h"
#include "pros/rtos.hpp"
#include <math.h>

constexpr double wheelDiameter = 2;                         // inches
constexpr double wheelCircumference = wheelDiameter * M_PI; // C = D * pi
constexpr double DEGREES_TO_INCHES = wheelCircumference / 360.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;

// --- Global variables for robot's position (Pose) ---
volatile double globalX = 0.0;
volatile double globalY = 0.0;
volatile double globalAngle = 0.0; // In degrees
volatile bool resetAngle = false;

/**
 * Task for Odometry Calculations.
 * Needs to be run fast to have an accurate position of robot
 */
void odometry_task() {
  while (imu.is_calibrating()) {
    pros::delay(20);
  }

  // Variables to store the previous sensor values
  double lastVertical = 0;
  double lastHorizontal = 0;
  double lastAngle = imu.get_rotation();

  while (true) {
    if (resetAngle) {
      resetAngle = false;
      imu.set_rotation(0);
      lastAngle = 0;
    }

    if (imu.is_calibrating()) {
      pros::delay(20);
    }

    // Get current sensor values
    double currentVertical = vertical_encoder.get_position() / 100.0;
    double currentHorizontal = horizontal_encoder.get_position() / 100.0;
    globalAngle = imu.get_rotation();

    // Calculate the delta in sensor values
    double deltaVertical = (currentVertical - lastVertical) * DEGREES_TO_INCHES;
    double deltaHorizontal =
        (currentHorizontal - lastHorizontal) * DEGREES_TO_INCHES;
    double deltaAngle = globalAngle - lastAngle;

    // Calculate the average heading for this loop cycle (more accurate)
    double avgAngleRad = (lastAngle + deltaAngle * 0.5) * DEG_TO_RAD;

    // Rotate the local movement vector to the global frame
    double deltaX = (deltaHorizontal * std::cos(avgAngleRad)) +
                    (deltaVertical * std::sin(avgAngleRad));
    double deltaY = (deltaHorizontal * std::sin(avgAngleRad)) +
                    (deltaVertical * std::cos(avgAngleRad));

    // Update the global position
    globalX += deltaX;
    globalY += deltaY;

    // Update the prev values for the next loop
    lastVertical = currentVertical;
    lastHorizontal = currentHorizontal;
    lastAngle = globalAngle;

    pros::delay(10);
  }
}