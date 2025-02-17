#include "globals.hpp"
#include "pros/optical.hpp"



constexpr Length wheelDiameter = 2.75_in;
constexpr AngularVelocity outputVelocity = 450_rpm;
constexpr LinearVelocity maxVel = toLinear<AngularVelocity>(outputVelocity, wheelDiameter);
constexpr Power power = 6 * 11_watt;
constexpr Mass mass = 8_kg;
constexpr LinearAcceleration maxAccel = power / maxVel / mass;
constexpr LinearAcceleration maxDeccel = maxAccel;

lib10478::VelocityController leftController(0.001853,0.000005,0.0000035,0.0442,0);
lib10478::VelocityController rightController(0.001853,0.000005,0.0000035,0.0442,0);
lib10478::Chassis chassis({-18,19,-20}, {-11,12,14}, new lemlib::V5InertialSensor{16,1.00485}, outputVelocity, wheelDiameter, {12.5_in, maxVel, maxAccel*0.5, maxDeccel*0.8, 0.15}, &leftController, &rightController);

lemlib::Motor intake(13,600_rpm);
pros::Motor arm(-2,pros::v5::MotorGears::red,pros::v5::MotorEncoderUnits::deg);

pros::adi::Pneumatics clamp('a',true);
pros::adi::Pneumatics intakePiston('c',false);
pros::adi::Pneumatics doinker('b',false);

pros::Optical optical(10);
lemlib::V5RotationSensor armSensor(1);
lib10478::PID armPID(12,0,20,0,false);