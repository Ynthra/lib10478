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

lib10478::TrackingWheel backTracker(new lemlib::V5RotationSensor{14},2.03058_in,0_in);
lemlib::V5InertialSensor imu(13,1.0068);

lemlib::V5RotationSensor armSensor(2);

pros::adi::Pneumatics clamp('d',true);
pros::adi::Pneumatics intakePiston('c',false);
pros::adi::Pneumatics lDoinker('a',false);
pros::adi::Pneumatics rDoinker('h',false);
pros::Optical colorSensor(3);

//lib10478::VelocityController linearController;
//lib10478::VelocityController angularController;
constexpr AngularVelocity driveSpeed = 450_rpm;
constexpr Length wheelDiameter = 3.262915_in;
constexpr LinearVelocity maxVel = toLinear<AngularVelocity>(driveSpeed,wheelDiameter);
constexpr Mass mass = 16_lb;
constexpr Length width = 12.3_in;
constexpr LinearAcceleration maxAccel = (66_watt / maxVel) / mass * 0.5;

// accel = force of 100rpm * (ratio) / mass
constexpr LinearAcceleration accel = from_N(toLinear<Torque>(0.6_Nm,wheelDiameter).internal()) * (100_rpm/driveSpeed) / (mass);
lib10478::Constraints constraints {.velLimits = maxVel, .maxAccel = 0.558_mps2, .maxDecel = 0.8644_mps2,.frictionCoefficent=0.2,.dd=0.2_cm};
lib10478::ProfileGenerator generator(lib10478::DISTANCE,constraints,width);

lib10478::PID armPID(0.05, 0, 0.2, 0, false);


/**
0.1, 0.14833
0.3, 0.5881
0.5, 1.041475
0.7, 1.48732
1.0, 2.10465
*/

/*

0.25, 1.94459
0.5, 5.537
0.75, 9.01355
1.0, 12.251
*/
lib10478::VelocityController linearController(0.458026, 0.14 , 0.06, 0.0280905, 0.8);
lib10478::VelocityController angularController(0.072533358, 0.02 , 0.008, 0.126115, 0.12);

/**lib10478::Chassis chassis(
    {-17,-16,15}, {-18,19,20}, //drive ports
    false,
    &imu,
    450_rpm, //drive speed
    width, //drive width
    wheelDiameter, //wheel diameter
    &linearController,&angularController,
    &generator,
    &backTracker
);**/
pros::MotorGroup leftgroup({-17,-16,15});
pros::MotorGroup rightgroup({-18,19,20});
lemlib::ControllerSettings linearSettings(
    14, //kP
    0, //ki
    35, //kd
    0, //windup range
    1, //small error range
    100, //small error timeout
    3, //large error range
    500, //large error timeout
    0);
lemlib::ControllerSettings angularSettings(
    4, //kP
    0, //ki
    35, //kd
    0, //windup range
    1, //small error range
    100, //small error timeout
    5, //large error range
    500, //large error timeout
    0);
lemlib::Drivetrain dt(&leftgroup,&rightgroup,12.3,3.262915,450,8);
pros::Rotation rotation(14);
pros::Imu imua(13);
lemlib::TrackingWheel wheel(&rotation,2.03058,0);
lemlib::Chassis chassis{
    dt,linearSettings,angularSettings,{nullptr,nullptr,&wheel,nullptr,&imua}
};

const Colour teamColour = BLUE;
