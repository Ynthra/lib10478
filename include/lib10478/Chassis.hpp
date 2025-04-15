#pragma once
#include "hardware/IMU/V5InertialSensor.hpp"
#include "lib10478/Odom.hpp"
#include "pros/rtos.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include <atomic>
#include <concepts>
#include <optional>
#include <utility>
#include "bezier.hpp"
#include "Profile.hpp"
#include "StateMachine.hpp"
#include "VelocityController.hpp"
#include "Math.hpp"
namespace lib10478
{

struct Constraints
{
    VelocityLimits velLimits;
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
    bool useRAMSETE = true;
    bool followReversed = false;
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
            bool swappedSides,
            lemlib::V5InertialSensor* imu,
            AngularVelocity outputVelocity,
            Length trackWidth,
            Length wheelDiameter, 
            VelocityController* linearController, VelocityController* angularController,
            TrackingWheel* backTracker);
    
    lemlib::MotorGroup leftMotors;
    lemlib::MotorGroup rightMotors;

    Profile* generateProfile(const virtualPath& path, Length dd = 0.2_cm, std::optional<Constraints> constraints = std::nullopt);
    void followProfile(Profile *profile, followParams params = {});
    void driveStraight(Length distance,followParams params = {});
    void turnTo(Angle angle, turnDirection direction = AUTO);
    void CancelMovement();
    void waitUntilSettled();
    void waitUntilDist(Length d);
    void init();
    void setPose(units::Pose pose);
    units::Pose getPose();

    void tank(AngularVelocity maxVel, double scale = 1.5);
    void moveVel(ChassisSpeeds speeds, std::pair<LinearVelocity, LinearVelocity> currentVel);

    std::pair<LinearVelocity, LinearVelocity> getVel();
    
    void move(Number left, Number right);

    void findWidth(Angle rotations);
    void findDiameter(Length distance);

    TrackingWheel rightTracker;
    TrackingWheel leftTracker;
private:
    bool swappedSides;
    Length distTarget = 0_m;
    Odom odom;
    lemlib::V5InertialSensor* imu;
    pros::Task* task = nullptr;
    pros::Mutex mutex;
    Profile *currentProfile = nullptr;
    std::atomic<bool> useRAMSETE = false;
    std::atomic<bool> followReversed = false;

    turnDirection direction;
    Angle targetAngle = 0_stDeg;
    VelocityController* linearController;
    VelocityController* angularController;

    Angle leftPrev = 0_stDeg;
    Angle rightPrev = 0_stDeg;
    Time timestamp = 0_msec;
    
    ChassisSpeeds RAMSETE(ChassisSpeeds speeds, units::Pose target, units::Pose current);
    std::pair<AngularVelocity, AngularVelocity> toMotorSpeeds(ChassisSpeeds speeds);
    void calibrateIMU();
    
    Length wheelDiameter;
    Length trackWidth;
};

}

inline std::vector<std::string> ouputs;