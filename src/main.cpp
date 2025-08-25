#include "main.h"
#include "config.h"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <math.h>

double wheelDiameter = 2;
double wheelCircumference = wheelDiameter * M_PI;            // C = D * pi
const double DEGREES_TO_INCHES = wheelCircumference / 360.0; // New constant
constexpr double DEG_TO_RAD = M_PI / 180.0;

// --- Global variables for robot's position (Pose) ---
volatile double globalX = 0.0;
volatile double globalY = 0.0;
volatile double globalAngle = 0.0; // In degrees

/**
 * Task for Odometry Calculations.
 * Runs at a high frequency to accurately track the robot's position.
 */
void odometry_task() {
  while (imu.is_calibrating()) {
    pros::delay(20);
  }

  // Variables to store the previous sensor values
  double lastVertical = 0;
  double lastHorizontal = 0;
  double lastAngle = imu.get_rotation();

  while (true) {
    // 1. Get current sensor values
    double currentVertical = vertical_encoder.get_position();
    double currentHorizontal = horizontal_encoder.get_position();
    globalAngle = imu.get_rotation();

    // 2. Calculate the change (delta) in sensor values
    double deltaVertical = (currentVertical - lastVertical) * DEGREES_TO_INCHES;
    double deltaHorizontal =
        (currentHorizontal - lastHorizontal) * DEGREES_TO_INCHES;
    double deltaAngle = globalAngle - lastAngle;

    // 3. Calculate the average heading for this loop cycle
    double avgAngleRad =
        (lastAngle + deltaAngle * 0.5) * DEG_TO_RAD; // Convert to Radians

    // 4. Rotate the local movement vector to the global frame
    double deltaX = (deltaHorizontal * std::cos(avgAngleRad)) -
                    (deltaVertical * std::sin(avgAngleRad));
    double deltaY = (deltaHorizontal * std::sin(avgAngleRad)) +
                    (deltaVertical * std::cos(avgAngleRad));

    // 5. Update the global position
    globalX += deltaX;
    globalY += deltaY;

    // 6. Update the "last" values for the next loop
    lastVertical = currentVertical;
    lastHorizontal = currentHorizontal;
    lastAngle = globalAngle;

    pros::delay(10);
  }
}

/**
 * Task for drawing to the LCD screen.
 * Its only responsibility is to display the robot's current pose.
 */
void lcd_loop_task() {
  pros::lcd::initialize();

  while (true) {
    // Check if the screen is working before trying to print
    if (pros::lcd::is_initialized()) {
      // Read the global variables that are updated by the odometry_task
      pros::lcd::print(0, "X: %.2f in", globalX);
      pros::lcd::print(1, "Y: %.2f in", globalY);
      pros::lcd::print(2, "Angle: %.2f deg", globalAngle);
    }

    // Update the screen every 50-100 milliseconds is plenty for a display
    pros::delay(50);
  }
}

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

// constants (tune to your robot/controller feel)
constexpr double CD_TURN_NONLINEARITY =
    0.5; // try 0.5–0.8 for smoother turn shaping
         // At the top with your other constants
const double TURN_REMAP_DENOMINATOR =
    sin(M_PI / 2 * CD_TURN_NONLINEARITY); // Calculated once!

constexpr double CD_NEG_INERTIA_SCALAR = 3.0; // responsiveness to turn changes
constexpr double CD_SENSITIVITY = 1.0;        // overall turn sensitivity
constexpr double DRIVE_DEADBAND = 0.02;       // ignore tiny joystick noise
constexpr double DRIVE_SLEW = 0.02;           // max throttle change per tick
constexpr double PI = M_PI;

// We apply a sinusoidal curve (twice) to the joystick input to give finer
// control at small inputs.
static double _turnRemapping(double iturn) {
  double firstRemapIteration =
      sin(M_PI / 2 * CD_TURN_NONLINEARITY * iturn) / TURN_REMAP_DENOMINATOR;
  return sin(M_PI / 2 * CD_TURN_NONLINEARITY * firstRemapIteration) /
         TURN_REMAP_DENOMINATOR;
}

// On each iteration of the drive controller (where we aren't point turning) we
// constrain the accumulators to the range [-1, 1].
double quickStopAccumlator = 0.0;
double negInertiaAccumlator = 0.0;
static void _updateAccumulators() {
  if (negInertiaAccumlator > 1) {
    negInertiaAccumlator -= 1;
  } else if (negInertiaAccumlator < -1) {
    negInertiaAccumlator += 1;
  } else {
    negInertiaAccumlator = 0;
  }

  if (quickStopAccumlator > 1) {
    quickStopAccumlator -= 1;
  } else if (quickStopAccumlator < -1) {
    quickStopAccumlator += 1;
  } else {
    quickStopAccumlator = 0.0;
  }
}

double prevTurn = 0.0;
double prevThrottle = 0.0;
std::pair<double, double> cheesyDrive(double ithrottle, double iturn) {
  bool turnInPlace = false;
  double linearCmd = ithrottle;
  if (fabs(ithrottle) < DRIVE_DEADBAND && fabs(iturn) > DRIVE_DEADBAND) {
    // The controller joysticks can output values near zero when they are
    // not actually pressed. In the case of small inputs like this, we
    // override the throttle value to 0.
    linearCmd = 0.0;
    turnInPlace = true;
  } else if (ithrottle - prevThrottle > DRIVE_SLEW) {
    linearCmd = prevThrottle + DRIVE_SLEW;
  } else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
    // We double the drive slew rate for the reverse direction to get
    // faster stopping.
    linearCmd = prevThrottle - (DRIVE_SLEW * 2);
  }

  double remappedTurn = _turnRemapping(iturn);

  double left, right;
  if (turnInPlace) {
    // The remappedTurn value is squared when turning in place. This
    // provides even more fine control over small speed values.
    left = remappedTurn * std::abs(remappedTurn);
    right = -remappedTurn * std::abs(remappedTurn);

  } else {
    double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
    negInertiaAccumlator += negInertiaPower;

    double angularCmd =
        abs(linearCmd) * // the more linear vel, the faster we turn
            (remappedTurn + negInertiaAccumlator) *
            CD_SENSITIVITY - // we can scale down the turning amount by a
                             // constant
        quickStopAccumlator;

    right = left = linearCmd;
    left += angularCmd;
    right -= angularCmd;

    _updateAccumulators();
  }

  prevTurn = iturn;
  prevThrottle = ithrottle;

  return std::make_pair(left, right);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  int mode = 0;
  while (true) {
    int raw_throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int raw_turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    double ithrottle = raw_throttle / 127.0;
    double iturn = raw_turn / 127.0;

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

    switch (mode) {
    case 0:
      top_motor.move(0);
      middle_motor.move(0);
      break;
    // Move block to back/storage
    case 1:
      middle_motor.move(0); // Explicitly stop the middle motor
      top_motor.move((intakeOn != 0 ? -intakeOn : 1) * 127);
      break;
    case 2:
      middle_motor.move((intakeOn != 0 ? -intakeOn : 1) * 127);
      top_motor.move((intakeOn != 0 ? intakeOn : 1) * 127);
      break;
    case 3:
      middle_motor.move((intakeOn != 0 ? intakeOn : 1) * 127);
      top_motor.move((intakeOn != 0 ? intakeOn : 1) * 127);
      break;
    }

    // Makes the robot drive
    auto [leftPower, rightPower] = cheesyDrive(ithrottle, iturn);
    left_mg.move(leftPower * -127);
    right_mg.move(rightPower * -127);

    pros::delay(20);
  }
}