#include "main.h"
#include "config.h"
#include "lcd.h"
#include "odometry.h"
#include "pros/rtos.hpp"
#include <math.h>

void initialize() {
  imu.reset(false);
  // CRITICAL STEP: Reset all odometry sensors to a known state of zero.
  vertical_encoder.reset_position();
  horizontal_encoder.reset_position();

  // Start the two tasks to run in the background
  pros::Task odom_task_handle(odometry_task, "Odometry Task");
  pros::Task lcd_task_handle(lcd_loop_task, "LCD Task");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol();