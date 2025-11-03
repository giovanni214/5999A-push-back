#include "config.h"
#include "liblvgl/llemu.hpp"
#include "odometry.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"


/**
 * Task for drawing to the LCD screen.
 * Purpose is to display the robot's current pose.
 */
void lcd_loop_task() {
  pros::lcd::initialize();

  // Check if the screen is working before trying to print
  while (!pros::lcd::is_initialized()) {
    pros::delay(50);
  }

  int dots = 1;
  const char *dotString = "..."; // max dots to print
  while (true) {
    // Read the global variables that are updated by the odometry_task
    if (imu.is_calibrating()) {
      pros::lcd::print(0, "Calibrating IMU%.*s", dots, dotString);
      dots = (dots + 1) % 4; // cycle 0–3
      pros::delay(100);
    } else {
      pros::lcd::print(0, "X: %.2f in", globalX);
      pros::lcd::print(1, "Y: %.2f in", globalY);
      pros::lcd::print(2, "Angle: %.2f deg", globalAngle);

    }

    pros::delay(50);
  }
}