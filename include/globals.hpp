#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Motor/Motor.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "lib10478/Chassis.hpp"
#include "lib10478/Odom.hpp"
#include "Hardware/Encoder/V5RotationSensor.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "lib10478/PID.hpp"
#include "lemlib/api.hpp"
#pragma once

extern lemlib::Chassis chassis;
extern lemlib::MotorGroup intake;
extern lemlib::Motor arm;

extern lib10478::TrackingWheel backTracker;
extern lemlib::V5InertialSensor imu;

extern lemlib::V5RotationSensor armSensor;

extern pros::adi::Pneumatics intakePiston;
extern pros::adi::Pneumatics rDoinker;
extern pros::adi::Pneumatics lDoinker;
extern pros::adi::Pneumatics clamp;

extern pros::Optical colorSensor;
extern lib10478::VelocityController linearController;
extern lib10478::VelocityController angularController;

extern lib10478::PID armPID;

extern lib10478::ProfileGenerator generator;

enum lbTargets{
	IDLE = 106,
	ALLIGNED = 83,
	RAISED = 60,
	SCORING = -40,
	HANGING = 0
};
inline lbTargets lbtarget = IDLE;

enum Colour {
    RED,
    BLUE,
    NONE
};

extern const Colour teamColour;
extern int timeStationary; 
extern bool disabledSort; 