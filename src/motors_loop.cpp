#include "config.h"
#include "pros/rtos.hpp"

volatile int intakeDir = 0;
volatile int mode = 0;

void motor_loop_task() {
  while (true) {
    intake_motor.move(intakeDir * 127);

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
        middle_motor.move(intakeDir * 127);
        top_motor.move(-intakeDir * 127);
      }
      break;
    case 2: // output in the middle
      middle_motor.move(intakeDir * 127);
      top_motor.move(intakeDir * 127);
      break;
    case 3: // output from the top
      middle_motor.move(intakeDir * 127);
      top_motor.move(-intakeDir * 127);
      break;
    case 4: // Move block to back/storage from BOTTOM
      gate_pneumatic.retract();
      middle_motor.move(-intakeDir * 127);
      top_motor.move(0);
      break;
    }

    pros::delay(50);
  }
}