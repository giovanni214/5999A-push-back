#include "main.h"
#include "config.h"
#include "lcd.h"
#include "liblvgl/llemu.hpp"
#include "motors_loop.h"
#include "odometry.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

void initialize() {
  pros::lcd::initialize();
  imu.reset(false);
  optical_sensor.set_led_pwm(100); // Turn on LED for consistent readings

  // CRITICAL STEP: Reset all odometry sensors to a known state of zero.
  vertical_encoder.reset_position();
  horizontal_encoder.reset_position();

  left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  // Start the two tasks to run in the background
  static pros::Task lcd_task_handle(lcd_loop_task, "LCD Task");
  static pros::Task odom_task_handle(odometry_task, "Odometry Task");
  static pros::Task motors_task_handle(motor_loop_task, "Motor Task");

  pros::lcd::print(0, "LCD is running: %d", lcd_task_handle.get_state());
}

void disabled() {}

void competition_initialize() {}

void autonomous();
void opcontrol();
