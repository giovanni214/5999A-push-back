#ifndef CONFIG_H
#define CONFIG_H

#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

extern pros::Controller controller;

//the stuff I actually care about
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::Motor intake_motor;
extern pros::Motor middle_motor;
extern pros::Motor top_motor;

extern pros::Imu imu;
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;


#endif // CONFIG_H