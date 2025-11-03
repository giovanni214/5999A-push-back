#include "config.h"
// #include "drive.h" // needed for cheesy drive
#include "auton.h"
#include "main.h"     // always include main.h
#include "odometry.h" // Provides globalX, globalY, globalAngle
#include "pid.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>

volatile double p = 200;
volatile double i = 1;
volatile double d = 0;

void opcontrol() {
  int mode = 0;
  // lock driving if still calibrating
  while (imu.is_calibrating())
    pros::delay(50);

  int maxUnloadTime = 50;
  int currentUnloadTime = 0;
  int nextLoopTime = 10;

  while (true) {
    // int raw_throttle =
    // controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); int raw_turn =
    // controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); double
    // ithrottle = raw_throttle / 127.0; double iturn = raw_turn / 127.0; Cheesy
    // Drive (Arcade Mode) auto [leftPower, rightPower] = cheesyDrive(ithrottle,
    // iturn);

    int leftPower = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightPower = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    int intakeOn = 0;
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
      intakeOn = 1;
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
      intakeOn = -1;

    intake_motor.move(intakeOn * 127);

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
      gate_pneumatic.toggle();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
      descore_pneumatic.toggle();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
      matchload_pneumatic.toggle();

    // working on it
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      mode = (mode == 1) ? 0 : 1; // If mode is 1, set to 0, otherwise set to 1
    }

    // output in the middle
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      mode = (mode == 2) ? 0 : 2;
    }

    // output from the top
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      mode = (mode == 3) ? 0 : 3;
    }

    // put into storage
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      mode = (mode == 4) ? 0 : 4;
    }

    switch (mode) {
    case 0:
      top_motor.move(0);
      middle_motor.move(0);
      break;

    case 1: // Move block to back/storage from TOP
      gate_pneumatic.extend();
      if (distance_sensor.get_distance() < 180) {
        middle_motor.move(127);
        top_motor.move(127);
      } else {
        middle_motor.move(intakeOn * 127);
        top_motor.move(-intakeOn * 127);
      }
      break;
    case 2: // output in the middle
      middle_motor.move(intakeOn * 127);
      top_motor.move(intakeOn * 127);
      break;
    case 3: // output from the top
      middle_motor.move(intakeOn * 127);
      top_motor.move(-intakeOn * 127);
      break;
    case 4: // Move block to back/storage from BOTTOM
      gate_pneumatic.retract();
      middle_motor.move(-intakeOn * 127);
      top_motor.move(0);
      break;
    }

    // Makes the robot drive
    left_mg.move(-leftPower);
    right_mg.move(-rightPower);

    // auton testing code
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      PIDController movingPID(p, i, d);
      PIDController turningPID(200, 1, 0); // set in stone for now
      globalX = 0;
      globalY = 0;
      resetAngle = true;

      intake_motor.move(127);
      moveForward(movingPID, 10);
    }

    if (currentUnloadTime >= maxUnloadTime) {
      currentUnloadTime = 0;
      matchload_pneumatic.toggle();
    } else if (currentUnloadTime != 0) {
      currentUnloadTime += nextLoopTime;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) &&
        matchload_pneumatic.is_extended() && currentUnloadTime == 0) {
      currentUnloadTime += nextLoopTime;
      matchload_pneumatic.toggle();
    }

    pros::delay(nextLoopTime);
  }
}