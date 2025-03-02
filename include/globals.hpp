#pragma once
#include "lib10478/Chassis.hpp"
#include "lib10478/Odom.hpp"
#include "lib10478/PID.hpp"
#include "lib10478/lib10478.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "units/units.hpp"


constexpr Length wheelDiameter = 2.776025236593060_in;
constexpr AngularVelocity outputVelocity = 450_rpm;
constexpr LinearVelocity maxVel = toLinear<AngularVelocity>(outputVelocity, wheelDiameter);
constexpr Power power = 6 * 11_watt;
constexpr Mass mass = 10_kg;
constexpr LinearAcceleration maxAccel = power / maxVel / mass;
constexpr LinearAcceleration maxDeccel = maxAccel;
constexpr Length trackWidth = 12.5_in;
extern lib10478::Constraints constraints;
extern lib10478::Chassis chassis;

extern lemlib::V5RotationSensor wheelSensor;
extern lib10478::TrackingWheel horizontalWheel;

extern lemlib::Motor intake;
extern pros::Motor arm;

extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics intakePiston;
extern pros::adi::Pneumatics ldoinker;
extern pros::adi::Pneumatics rdoinker;

extern pros::Optical optical;
extern lemlib::V5RotationSensor armSensor;
extern lib10478::PID armPID;

enum lbTargets{
	DOWN = -118,
	ALLIGNED = -92,
	RAISED = -60,
	SCORING = 32,
	HANGING = 0
};
inline lbTargets lbtarget = DOWN;

#define TEAMCOLOR RED