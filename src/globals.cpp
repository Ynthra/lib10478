#include "globals.hpp"
#include "Hardware/Encoder/V5RotationSensor.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lib10478/PID.hpp"
#include "lib10478/ProfileGenerator.hpp"
#include "lib10478/VelocityController.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"


lemlib::MotorGroup intake({11,-12},600_rpm);
lemlib::Motor arm(1,100_rpm);

lemlib::V5InertialSensor imu(13,1.0068);

lemlib::V5RotationSensor armSensor(2);

pros::adi::Pneumatics clamp('d',true);
pros::adi::Pneumatics intakePiston('c',false);
pros::adi::Pneumatics lDoinker('a',false);
pros::adi::Pneumatics rDoinker('h',false);
pros::Optical colorSensor(3);


lib10478::PID armPID(0.05, 0, 0.2, 0, false);

pros::MotorGroup leftgroup({-17,-16,15});
pros::MotorGroup rightgroup({-18,19,20});
lemlib::ControllerSettings linearSettings(
    14, //kP
    0, //ki
    38, //kd
    0, //windup range
    1, //small error range
    100, //small error timeout
    3, //large error range
    1000, //large error timeout
    0);
lemlib::ControllerSettings angularSettings(
    3, //kP
    0, //ki
    30, //kd
    0, //windup range
    1, //small error range
    100, //small error timeout
    3, //large error range
    300, //large error timeout
    20);

lemlib::Drivetrain dt(&leftgroup,&rightgroup,12.3,3.262915,450,8);
pros::Rotation rotation(14);
pros::Imu imua(13);
lemlib::TrackingWheel wheel(&rotation,2.03058,0);

bool flipped = false; // false for right side, true for left side
const Colour teamColour = BLUE;
lemlib::Chassis chassis{
    dt,linearSettings,angularSettings,{nullptr,nullptr,&wheel,nullptr,&imua},
    flipped
};

