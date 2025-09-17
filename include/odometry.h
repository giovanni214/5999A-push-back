#pragma once

// Global robot position variables
extern volatile double globalX;
extern volatile double globalY;
extern volatile double globalAngle;
extern volatile bool resetAngle;

// Odometry background task
void odometry_task();
