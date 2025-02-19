#include "Chassis.hpp"
#include "Profile.hpp"
#include "bezier.hpp"
#include "hardware/Port.hpp"
#include "lib10478/Odom.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "pros/adi.h"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include <cmath>
#include <compare>
#include <cstdint>
#include <cstdio>
#include <math.h>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include "lib10478/controller.hpp"
#include "lib10478/Math.hpp"


using namespace lib10478;

Chassis::Chassis(std::initializer_list<lemlib::ReversibleSmartPort> leftPorts,
                 std::initializer_list<lemlib::ReversibleSmartPort> rightPorts,
                 lemlib::IMU* imu,
                 AngularVelocity outputVelocity,
                 Length wheelDiameter, Constraints constraints, 
                 VelocityController* leftController, VelocityController* rightController,
                 TrackingWheel* backTracker)
    : leftMotors(leftPorts,outputVelocity), rightMotors(rightPorts,outputVelocity), 
      wheelDiameter(wheelDiameter), constraints(constraints),
      leftTracker(&leftMotors,wheelDiameter,-constraints.trackWidth/2),
      rightTracker(&rightMotors,wheelDiameter,constraints.trackWidth/2),
      imu(imu),
      odom(imu, &leftTracker, &rightTracker, backTracker), 
      leftController(leftController),rightController(rightController) {}

LinearVelocity Chassis::maxSpeed(Curvature curvature){
    const LinearVelocity maxSlip = units::sqrt(9.81_mps2 * constraints.frictionCoefficent / curvature);
    const LinearVelocity maxTurn = (2*constraints.maxVel) / (units::abs(curvature)*constraints.trackWidth+2);
    return units::min(maxSlip,maxTurn);
}

Profile* Chassis::generateProfile(const virtualPath& path, Length dd)
{
    Length dist = dd;
    LinearVelocity vel = 0_mps;
    double t = 0;

    std::vector<ProfilePoint> profile;

    while (t <= 1)
    {
        const units::V2Position point = path.getPoint(t);
        const units::V2Position deriv = path.getDerivative(t);
        const Curvature curvature = path.getCurvature(t);
        
        t += dd.internal() / deriv.magnitude().internal();

        const LinearAcceleration maxAccel = (2*this->constraints.maxAccel)/(units::abs(curvature)*constraints.trackWidth+2);
        vel = std::min(maxSpeed(curvature), units::sqrt(vel * vel + 2 * maxAccel * dd));
        dist += dd;

        profile.push_back(ProfilePoint({point,units::atan2(deriv.y, deriv.x)}, vel, curvature));
    }

    vel = 0_mps;

    for (int i = profile.size()-1; i >= 0; i--){
        const ProfilePoint profilePoint = profile[i];
        const LinearAcceleration maxDecel = (2*this->constraints.maxDecel)/(units::abs(profilePoint.curvature)*constraints.trackWidth+2);
        vel = std::min(profilePoint.velocity, units::sqrt(vel * vel + 2 * maxDecel * dd));
        profile[i].velocity = vel;
    }

    return new Profile(profile,dd);
}
//v = wr
//vc = w
std::pair<AngularVelocity, AngularVelocity> Chassis::toMotorSpeeds(ChassisSpeeds speeds)
{
	const LinearVelocity velLeft = speeds.v - toLinear<AngularVelocity>(speeds.ω, constraints.trackWidth);
	const LinearVelocity velRight = speeds.v + toLinear<AngularVelocity>(speeds.ω, constraints.trackWidth);

    return std::make_pair(toAngular<LinearVelocity>(velLeft,wheelDiameter),
                          toAngular<LinearVelocity>(velRight,wheelDiameter));
}

void Chassis::followProfile(Profile *profile, followParams params)
{
    std::lock_guard<pros::Mutex> lock(mutex);
    profile->prev = 0;
    currentProfile = profile;
    this->useRAMSETE = params.useRAMSETE;
    this->followReversed = params.followReversed;
    setState(ChassisState::FOLLOW);
}

constexpr Angle sanitizeAngle(Angle angle) {
    return units::mod(units::mod(angle, 1_stRot) + 1_stRot, 1_stRot);
}
Angle angleError(Angle target, Angle position, turnDirection direction) {
    // bound angles from 0 to 2pi or 0 to 360
    target = sanitizeAngle(target);
    target = sanitizeAngle(target);
    const Angle max = 1_stRot;
    const Angle rawError = target - position;
    switch (direction) {
        case lib10478::CW: // turn clockwise
            if(rawError.internal() < 0) return rawError + max;
            else return rawError;// add max if sign does not match
        case lib10478::CCW: // turn counter-clockwise
            if(rawError.internal() > 0) return rawError - max;
            else return rawError;// ad // subtract max if sign does not match
        default: // choose the shortest path
            return units::remainder(rawError, max);
    }
}
Angle Chassis::getError(Angle target, Angle position, turnDirection direction) {
    // Wrap the angle to be within 0pi and 2pi radians
    target = units::mod(units::mod(target, 1_stRot) + 1_stRot, 1_stRot);
    position = units::mod(units::mod(position, 1_stRot) + 1_stRot, 1_stRot);

    Angle error = target - position;
    if (direction == AUTO) return from_stDeg(std::remainder(to_stDeg(error), 360));
    if (direction == CCW) return error < 0_stRot ? error + 1_stRot : error;
    else return error > 0_stRot ? error - 1_stRot : error;
}
int turnLoops;
void Chassis::turnTo(Angle angle, turnDirection direction){
    std::lock_guard<pros::Mutex> lock(mutex);
    const units::Pose pose = odom.getPose();
    const Length distance = toLinear<Angle>(getError(angle, pose.orientation, direction),this->constraints.trackWidth);
    this->direction = turnDirection(sgn(distance.internal()));
    this->targetAngle = angle;
    currentProfile = this->generateProfile(CubicBezier({0_m,0_m},{0_m,0.33*distance},{0_m,0.66*distance},{0_m,distance}));
    setState(ChassisState::TURN);
}

void Chassis::CancelMovement()  
{
    std::lock_guard<pros::Mutex> lock(mutex);
    tank(0_percent,0_percent);
    setState(ChassisState::IDLE);
}
void Chassis::waitUntilSettled()
{
    auto now = pros::millis();
    while (getState() != ChassisState::IDLE) {
        pros::Task::delay_until(&now, 10);
    }
}
void Chassis::setPose(units::Pose pose){
    std::lock_guard<pros::Mutex> lock(mutex);
    odom.setPose(pose);
}

units::Pose Chassis::getPose(){
    std::lock_guard<pros::Mutex> lock(mutex);
    return odom.getPose();
}

void Chassis::tank(Number left, Number right){
    if (getState() == ChassisState::IDLE){
        leftMotors.move(left);
        rightMotors.move(right);
    }
}

//dTheta = (dL-dR)/width
//width = (dL-dR)/dTheta
void Chassis::findWidth(Angle rotations){
    this->leftMotors.setAngle(0_stDeg);
    this->rightMotors.setAngle(0_stDeg);

    while (true) {
        controller::update();
        if(controller::Right.pressing){
            this->leftMotors.move(50_percent);
            this->rightMotors.move(-50_percent);
        }
        else if(controller::Left.pressing){
            this->leftMotors.move(-50_percent);
            this->rightMotors.move(50_percent);
        }
        else {
            this->leftMotors.move(0_percent);
            this->rightMotors.move(0_percent);
        }
        controller::master.set_text(2,0,std::to_string(
            ((toLinear<Angle>(this->leftMotors.getAngle(),this->wheelDiameter) - 
            toLinear<Angle>(this->rightMotors.getAngle(),this->wheelDiameter))/
            rotations.internal()).convert(in)));
        if(controller::A.pressed) break;
        pros::delay(50);
    }
}

// s = rTheta
// 2s/Theta = diameter

void Chassis::findDiameter(Length distance){
    this->leftMotors.setAngle(0_stDeg);
    this->rightMotors.setAngle(0_stDeg);
    while (true) {
        controller::update();
        controller::master.set_text(2,0,std::to_string(2*distance.convert(in)/
        ((this->leftMotors.getAngle().internal() + this->rightMotors.getAngle().internal())/2)));
        if(controller::A.pressed) break;
        pros::delay(50);
    }
}

ChassisSpeeds Chassis::RAMSETE(ChassisSpeeds speeds, units::Pose target, units::Pose current)
{
    const double zeta = 0.7;
    const double beta = 2;
    units::Pose localError = (target - current).rotatedBy(-current.orientation);
    localError = {localError.x,localError.y,from_stRad(std::remainder(localError.orientation.internal(),2*M_PI))};
    
    
    // k = 2ζ√(ω² + b v²)
    const double k = 2.0 * zeta * std::sqrt(std::pow(speeds.ω.internal(), 2) + beta * std::pow(speeds.v.internal(), 2));

    // v_cmd = v cos(e_θ) + k e_x
    // ω_cmd = ω + k e_θ + b v sinc(e_θ) e_y
    const LinearVelocity adjustedLinear = from_mps(speeds.v.internal() * std::cos(localError.orientation.internal())
                                                    + k * localError.x.internal());
    const AngularVelocity adjustedAngular = from_radps(speeds.ω.internal() 
                                                    + k * localError.orientation.internal()
                                                    + beta * speeds.v.internal() 
                                                    * sinc(localError.orientation.internal())
                                                    * localError.y.internal());
    
    return ChassisSpeeds {adjustedAngular,adjustedLinear};
}

void Chassis::init() {
    if (task == nullptr) {
        imu->calibrate();
        while(imu->isCalibrating()){
            pros::delay(10);
        }
        if(imu->isConnected()){
            controller::master.rumble(".");
        }
        else{
            controller::master.rumble("..");
        }
        odom.setPose({0_m,0_m,0_cDeg});
        task = new pros::Task ([this] {

            std::uint32_t now = pros::millis();

            std::pair<Angle,Angle> lastAngles = {leftMotors.getAngle(),rightMotors.getAngle()};

            SimpleMovingAverage leftsma(2);
            SimpleMovingAverage rightsma(2);
            int loops = 0;
            std::pair<AngularVelocity, AngularVelocity> motorSpeeds = {0_rpm,0_rpm};
            int time = 0;
            while (true) {
                mutex.take();
                odom.update();
                const std::pair<Angle,Angle> currentAngles = {leftMotors.getAngle(),rightMotors.getAngle()};
                const std::pair<AngularVelocity,AngularVelocity> currentVelocity = {
                    (currentAngles.first - lastAngles.first) / (10_msec),
                    (currentAngles.second - lastAngles.second) / (10_msec)
                };
                //controller::master.set_text(2,0,std::to_string((leftsma.next(currentVelocity.first.convert(rpm) +
                //                                                rightsma.next(currentVelocity.second.convert(rpm))))*0.5));        
                if (motorSpeeds.first != 0_rpm ) ouputs.push_back(std::to_string(time) + "," + std::to_string(currentVelocity.first.convert(rpm)) + "," + std::to_string(motorSpeeds.first.convert(rpm)));
                switch (getState()) {
                
                case ChassisState::IDLE:
                    //controller::master.set_text(0,0,"idle         ");
                    motorSpeeds = {0_rpm,0_rpm};
                    break;
                
                case ChassisState::FOLLOW: {
                    //controller::master.set_text(0,0,"following   ");
                    units::Pose currentPose = odom.getPose();
                    if (followReversed) currentPose.orientation = currentPose.orientation + from_stDeg(M_PI);
                    const ProfilePoint point = currentProfile->getProfilePoint(currentPose);
                    if (point == currentProfile->profile.back()) {
                        setState(ChassisState::IDLE);
                        tank(0_percent,0_percent);
                        break;
                    }
                    
                    ChassisSpeeds speeds = {
                        from_radps(point.velocity.internal() * point.curvature.internal()),
                        point.velocity
                    };
                    
                    if(followReversed) speeds.v = -speeds.v;

                    if (useRAMSETE) {
                        speeds = RAMSETE(speeds, point.pose, odom.getPose());
                    }

                    motorSpeeds = toMotorSpeeds(speeds);

                    if(leftController == nullptr || rightController == nullptr){
                        leftMotors.moveVelocity(motorSpeeds.first);
                        rightMotors.moveVelocity(motorSpeeds.second);
                    }
                    else{
                        Number leftPower = leftController->getPower(motorSpeeds.first,currentVelocity.first);
                        Number rightPower = rightController->getPower(motorSpeeds.second,currentVelocity.second);
                        const Number maxPower = std::max(leftPower,rightPower);
                        if(maxPower > 1_num){
                            leftPower = leftPower/maxPower;
                            rightPower = rightPower/maxPower;
                        }
                        leftMotors.move(leftPower);
                        rightMotors.move(rightPower);
                    }
                    break;
                }
                
                case ChassisState::TURN:
                    units::Pose currentPose = odom.getPose();
                    const Angle angularError = getError(this->targetAngle,currentPose.orientation,this->direction);
                    const Length d = currentProfile->getLength() - this->direction*toLinear<Angle>(
                                angularError, this->constraints.trackWidth);

                    if (currentProfile == nullptr) break;
                    LinearVelocity velocity = currentProfile->getProfilePoint(d).velocity;

                    if (units::abs(angularError) > 350_stDeg) {
                        delete currentProfile;
                        setState(ChassisState::IDLE);
                        tank(0_percent,0_percent);
                        break;
                    }
                    if(this->direction == CW) velocity = -velocity;
                    AngularVelocity motorVel = toAngular<LinearVelocity>(velocity,wheelDiameter);
                    
                    motorSpeeds = {-motorVel,motorVel};
                    if(leftController == nullptr || rightController == nullptr){
                        leftMotors.moveVelocity(motorSpeeds.first);
                        rightMotors.moveVelocity(motorSpeeds.second);
                    }
                    else{
                        Number leftPower = leftController->getPower(motorSpeeds.first,currentVelocity.first);
                        Number rightPower = rightController->getPower(motorSpeeds.second,currentVelocity.second);
                        const Number maxPower = std::max(leftPower,rightPower);
                        if(maxPower > 1_num){
                            leftPower = leftPower/maxPower;
                            rightPower = rightPower/maxPower;
                        }
                        leftMotors.move(leftPower);
                        rightMotors.move(rightPower);
                    }
                    break;
                }

                lastAngles = currentAngles;
                mutex.give();
                pros::Task::delay_until(&now, 10);
                time += 10;
            }
        },TASK_PRIORITY_MAX-3);
    }
}