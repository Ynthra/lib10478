#pragma once
#include "lib10478/Odom.hpp"
#include "pros/rtos.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include <atomic>
#include <concepts>
#include <utility>
#include "bezier.hpp"
#include "Profile.hpp"
#include "StateMachine.hpp"
#include "VelocityController.hpp"
namespace lib10478
{

struct Constraints
{
    Length trackWidth;
    LinearVelocity maxVel;
    LinearAcceleration maxAccel;
    LinearAcceleration maxDecel;
    Number frictionCoefficent;
};

struct ChassisSpeeds{
    AngularVelocity ω;
    LinearVelocity v;
};

struct followParams
{
    bool useRAMSETE = false;
    bool followReversed = false;
};

enum turnDirection{
    CW = -1,
    CCW = 1,
    AUTO = 0
};

enum class ChassisState{
    IDLE, FOLLOW, TURN
};
// forward declare
template class StateMachine<ChassisState,ChassisState::IDLE>;

class Chassis : public StateMachine<ChassisState, ChassisState::IDLE>
{

public:
    Chassis(std::initializer_list<lemlib::ReversibleSmartPort> leftPorts,
            std::initializer_list<lemlib::ReversibleSmartPort> rightPorts,
            lemlib::IMU* imu,
            AngularVelocity outputVelocity,
            Length wheelDiameter, Constraints constraints, 
            VelocityController* leftController, VelocityController* rightController,
            TrackingWheel* backTracker);
    
    lemlib::MotorGroup leftMotors;
    lemlib::MotorGroup rightMotors;

    Profile* generateProfile(const virtualPath& path, Length dd = 0.2_cm);
    void followProfile(Profile *profile, followParams params = {});
    void turnTo(Angle angle, turnDirection direction = AUTO);
    void CancelMovement();
    void waitUntilSettled();
    void init();
    void setPose(units::Pose pose);
    units::Pose getPose();

    void tank(Number left, Number right);
    void findWidth(Angle rotations);
    void findDiameter(Length distance);

    TrackingWheel rightTracker;
    TrackingWheel leftTracker;
private:
    Odom odom;
    lemlib::IMU* imu;
    pros::Task* task = nullptr;
    pros::Mutex mutex;
    Profile *currentProfile = nullptr;
    std::atomic<bool> useRAMSETE = false;
    std::atomic<bool> followReversed = false;

    turnDirection direction;
    Angle targetAngle = 0_stDeg;
    VelocityController* leftController;
    VelocityController* rightController;

    Angle getError(Angle target, Angle curent, turnDirection direction);
    LinearVelocity maxSpeed(Curvature Curvature);
    ChassisSpeeds RAMSETE(ChassisSpeeds speeds, units::Pose target, units::Pose current);
    std::pair<AngularVelocity, AngularVelocity> toMotorSpeeds(ChassisSpeeds speeds);
    Length wheelDiameter;
    Constraints constraints;
};

}

inline std::vector<std::string> ouputs;