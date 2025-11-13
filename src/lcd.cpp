#include "config.h"
#include "odometry.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>


/**
 * Task for drawing to the LCD screen.
 * Purpose is to display the robot's current pose.
 */
void lcd_loop_task() {
  int dots = 1;
  const char *dotString = "...";
  
  while (true) {
    if (imu.is_calibrating()) {
      pros::lcd::print(0, "Calibrating IMU%.*s", dots, dotString);
      dots = (dots + 1) % 4;
      pros::delay(100);
    } else {
      pros::lcd::print(0, "X: %.2f in", globalX);
      pros::lcd::print(1, "Y: %.2f in", globalY);
      pros::lcd::print(2, "Angle: %.2f deg", globalAngle);
    }

    pros::delay(50);
  }
}