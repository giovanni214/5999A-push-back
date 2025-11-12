#include "config.h"
#include "odometry.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"


/**
 * Task for drawing to the LCD screen.
 * Purpose is to display the robot's current pose.
 */
void lcd_loop_task() {
  // Check if the screen is working before trying to prin
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

    // pros::c::optical_rgb_s_t rgb = optical_sensor.get_rgb();
    // pros::lcd::print(3, "R: %d", (int)(rgb.red * 255));
    // pros::lcd::print(4, "G: %d", (int)(rgb.green * 255));
    // pros::lcd::print(5, "B: %d", (int)(rgb.blue * 255));
    pros::delay(50);
  }
}