#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"


// Defines controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Creates a motor group with forwards
// starting from back
pros::MotorGroup left_mg({14, 13, 12}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({-17, -18, -16}, pros::MotorGearset::blue);

// create the imu (inertial sensor)
pros::Imu imu(11);

pros::Distance distance_sensor(8);

pros::Rotation horizontal_encoder(15);
pros::Rotation vertical_encoder(19);

pros::adi::Pneumatics gate_pneumatic(2, true);
pros::adi::Pneumatics descore_pneumatic(1, false);

pros::Motor intake_motor(-20, pros::v5::MotorGears::green);
pros::Motor top_motor(9, pros::v5::MotorGears::green);
pros::Motor middle_motor(2, pros::v5::MotorGears::green);
