#include "config.h"
// #include "drive.h" // needed for cheesy drive
#include "main.h" // always include main.h
#include "motors_loop.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>

void opcontrol() {
  // lock driving if still calibrating
  while (imu.is_calibrating())
    pros::delay(50);

  int maxUnloadTime = 50;
  int currentUnloadTime = 0;

  int maxPunchTime = 100;
  int currentPunchTime = 0;
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

    intakeDir = 0;
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
      intakeDir = 1;
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
      intakeDir = -1;

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
      gate_pneumatic.toggle();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
      descore_pneumatic.toggle();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
      matchload_pneumatic.toggle();

    // working on it
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
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
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      mode = (mode == 4) ? 0 : 4;
    }

    // Makes the robot drive
    left_mg.move(-leftPower);
    right_mg.move(-rightPower);

    //match load pnuematic pulser
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

    //puncher inside of basket pulser
    if (currentPunchTime >= maxPunchTime) {
      currentPunchTime = 0;
      punch_pneumatic.toggle();
    } else if (currentPunchTime != 0) {
      currentPunchTime += nextLoopTime;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) &&
        !punch_pneumatic.is_extended() && currentPunchTime == 0) {
      currentPunchTime += nextLoopTime;
      punch_pneumatic.toggle();
    }

    pros::delay(nextLoopTime);
  }
}