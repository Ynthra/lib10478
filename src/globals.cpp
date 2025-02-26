#include "globals.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "lib10478/Chassis.hpp"
#include "lib10478/Odom.hpp"
#include "lib10478/Profile.hpp"
#include "lib10478/VelocityController.hpp"
#include "pros/optical.hpp"

lemlib::V5RotationSensor wheelSensor(17);
//42.5
lib10478::TrackingWheel horizontalWheel(&wheelSensor,-2.0207188_in,-0.2_in);

lib10478::Constraints constraints = {maxVel, maxAccel*0.5, maxDeccel*0.8, 0.15}; 
lib10478::VelocityController leftController(0.001853,0.000005,0.0000035,0.0442,0.0025);
lib10478::VelocityController rightController(0.001853,0.000005,0.0000035,0.0442,0.0025);
lib10478::Chassis chassis({-18,19,-20}, {-11,12,14}, new lemlib::V5InertialSensor{9,1.00485}, outputVelocity,trackWidth ,wheelDiameter, constraints , &leftController, &rightController, &horizontalWheel);

lemlib::Motor intake(13,600_rpm);
pros::Motor arm(-2,pros::v5::MotorGears::red,pros::v5::MotorEncoderUnits::deg);

pros::adi::Pneumatics clamp('a',true);
pros::adi::Pneumatics intakePiston('c',false);
pros::adi::Pneumatics ldoinker('b',false);
pros::adi::Pneumatics rdoinker('d',true);

pros::Optical optical(10);
lemlib::V5RotationSensor armSensor(1);
lib10478::PID armPID(6,0,10,0,false);