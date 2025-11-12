#ifndef CONFIG_H
#define CONFIG_H

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

extern pros::Controller controller;

// the stuff I actually care about
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::Motor intake_motor;
extern pros::Motor middle_motor;
extern pros::Motor top_motor;

extern pros::Imu imu;
extern pros::Optical optical_sensor;
extern pros::Distance distance_sensor;
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;

extern pros::adi::Pneumatics gate_pneumatic;
extern pros::adi::Pneumatics descore_pneumatic;
extern pros::adi::Pneumatics matchload_pneumatic;

#endif // CONFIG_H