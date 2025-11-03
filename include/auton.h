#pragma once

#include "pid.h"

static double shortestAngleDiff(double target, double current);

void turnToAngle(PIDController &PID, double targetAngle, double tolerance = 1,
                 bool debugMode = true);

void moveForward(PIDController &movePID, double targetDistance,
                 double tolerance = 1, bool debugMode = true);

void turnAndMoveToPoint(PIDController &turnPID, PIDController &movePID,
                        double x, double y, double turnTolerance = 1,
                        double moveTolerance = 1, bool debugMode = true);
