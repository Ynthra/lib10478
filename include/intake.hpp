#pragma once
#include "globals.hpp"

extern Colour getColour();
extern void detectRing();
extern void sort();

inline bool isSorting = false;

#define numTeeth 62.0
#define outSprocket 12.0
#define numHooks 3.0
constexpr double hookDist = numTeeth/outSprocket/numHooks*360;
inline lemlib::Motor topIntake(11,600_rpm);
#define SETPOINT(x) from_stDeg(roundUpToNearestMultiple(to_stDeg(topIntake.getAngle()- from_stDeg(hookDist) * x), hookDist, 0) + hookDist * x)