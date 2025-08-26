#pragma once

// Global robot position variables
extern volatile double globalX;
extern volatile double globalY;
extern volatile double globalAngle;

// Odometry background task
void odometry_task();
