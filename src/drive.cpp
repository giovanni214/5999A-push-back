// constants (tune to your robot/controller feel)
#include <math.h>
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