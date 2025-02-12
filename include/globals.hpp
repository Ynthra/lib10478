#pragma once
#include "lib10478/PID.hpp"
#include "lib10478/lib10478.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"

extern lib10478::Chassis chassis;

extern lemlib::Motor intake;
extern pros::Motor arm;

extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics intakePiston;
extern pros::adi::Pneumatics doinker;

extern pros::Optical optical;
extern lemlib::V5RotationSensor armSensor;
extern lib10478::PID armPID;

#define TEAMCOLOR RED