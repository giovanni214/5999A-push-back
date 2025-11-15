#pragma once

extern volatile int intakeDir;
extern volatile int mode;
extern volatile bool extendGate;

void motor_loop_task();
