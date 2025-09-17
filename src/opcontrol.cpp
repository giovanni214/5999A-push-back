#include "config.h"
// #include "drive.h" needed for cheesy drive
#include "main.h" // always include main.h
#include "pros/misc.h"

void opcontrol() {
  int mode = 0;
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

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      mode = (mode == 1) ? 0 : 1; // If mode is 1, set to 0, otherwise set to 1
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      mode = (mode == 2) ? 0 : 2;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      mode = (mode == 3) ? 0 : 3;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      mode = (mode == 4) ? 0 : 4;
    }

    switch (mode) {
    case 0:
      top_motor.move(0);
      middle_motor.move(0);
      break;
    // Move block to back/storage
    case 1:
      middle_motor.move(0); // Explicitly stop the middle motor
      top_motor.move(-intakeOn * 127);
      break;
    case 2:
      middle_motor.move(intakeOn * 127);
      top_motor.move(intakeOn * 127);
      break;
    case 3:
      middle_motor.move(intakeOn * 127);
      top_motor.move(-intakeOn * 127);
      break;
    case 4:
      middle_motor.move(-intakeOn * 127);
      top_motor.move(0);
      break;
    }

    // Makes the robot drive
    left_mg.move(-leftPower);
    right_mg.move(-rightPower);

    pros::delay(20);
  }
}