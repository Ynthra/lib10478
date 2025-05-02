#include "Chassis.hpp"
#include "Profile.hpp"
#include "controller.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Port.hpp"
#include "lib10478/Odom.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <mutex>
#include <utility>
#include "lib10478/Math.hpp"


using namespace lib10478;

Chassis::Chassis(std::initializer_list<lemlib::ReversibleSmartPort> leftPorts,
                 std::initializer_list<lemlib::ReversibleSmartPort> rightPorts,
                 bool swappedSides, 
                 lemlib::V5InertialSensor* imu,
                 AngularVelocity outputVelocity,
                 Length trackWidth,
                 Length wheelDiameter, 
                 VelocityController* linearController, VelocityController* angularController,
                 ProfileGenerator* angularGenerator,
                 TrackingWheel* backTracker)
    : leftMotors(leftPorts,outputVelocity), rightMotors(rightPorts,outputVelocity), 
      swappedSides(swappedSides),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      leftTracker(&leftMotors,wheelDiameter,-trackWidth/2),
      rightTracker(&rightMotors,wheelDiameter,trackWidth/2),
      imu(imu),
      odom(imu, &leftTracker, &rightTracker, backTracker), 
      linearController(linearController),angularController(angularController),
      angularGenerator(angularGenerator) {}

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

void Chassis::driveStraight(Length distance,followParams params){
    auto pose = odom.getPose();
    units::Vector2D<Number> direction = units::Vector2D<Number>::fromPolar(pose.orientation, 1);
    /**const auto profile =  this->generateProfile(CubicBezier(pose,
                                                       pose + 0.33 * distance * direction,
                                                       pose + 0.66 * distance * direction,
                                                       pose + distance * direction),0.1_cm);**/
    //followProfile(profile,params);
}
void Chassis::followProfile(Profile *profile, followParams params)
{
    std::lock_guard<pros::Mutex> lock(mutex);
    distTarget = profile->getLength();
    profile->prev = 0;
    currentProfile = profile;
    this->useRAMSETE = params.useRAMSETE;
    this->followReversed = params.followReversed;
    setState(ChassisState::FOLLOW);
}



int turnLoops;
void Chassis::turnTo(Angle angle, turnDirection direction){
    std::lock_guard<pros::Mutex> lock(mutex);
    const units::Pose pose = odom.getPose();
    const Length distance = toLinear<Angle>(getAngularError(angle, pose.orientation, this->direction),this->trackWidth);
    this->direction = turnDirection(sgn(distance.internal()));
    this->targetAngle = angle;
    currentProfile = this->angularGenerator->generateProfile(CubicBezier({0_m,0_m},{0_m,0.33*distance},{0_m,0.66*distance},{0_m,distance}));
    setState(ChassisState::TURN);
}

void Chassis::waitUntilDist(Length d){
    auto now = pros::millis();
    while(true){
        std::lock_guard<pros::Mutex> lock(mutex);
        if(this->distTarget < d) break;
        pros::Task::delay_until(&now, 10);
    }
}

void Chassis::CancelMovement()  
{
    std::lock_guard<pros::Mutex> lock(mutex);
    setState(ChassisState::IDLE);
    move(0_percent,0_percent);
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
    this->odom.setPose(pose);
}

units::Pose Chassis::getPose(){
    std::lock_guard<pros::Mutex> lock(mutex);
    auto pose = this->odom.getPose();
    return pose;
}

/*
* (swap sign for ccw)
*
*  left = throttle + turn
*  right = throttle - turn
*  
*  throttle = (left + right)/2
*  turn = (left - right)/2
*
*/
void Chassis::updateVel(){
    const Time now = from_msec(pros::millis());
    const Time dt =  now - this->timestamp;

    const Angle leftAngle = leftMotors.getAngle();
    const Angle rightAngle = rightMotors.getAngle();

    const LinearVelocity leftVel = toLinear<AngularVelocity>((leftAngle-this->leftPrev)/dt,this->wheelDiameter);
    const LinearVelocity rightVel = toLinear<AngularVelocity>((rightAngle-this->rightPrev)/dt,this->wheelDiameter);


    if (leftAngle != leftPrev || rightAngle != rightPrev || dt > 50_msec) this->timestamp = now;

    this->leftPrev = leftAngle;
    this->rightPrev = rightAngle;

    currentVel = {leftVel, rightVel};
}   

void Chassis::tank(AngularVelocity maxVel, double scale){
    const LinearVelocity linearMax = toLinear<AngularVelocity>(maxVel,this->wheelDiameter);
    const double left = Controller::master[LEFT_Y];
    const double right = Controller::master[RIGHT_Y];
    const double leftScaled = driveCurve(left * (fabs(left) > 1/127.0), scale);
    const double rightScaled = driveCurve(right * (fabs(right) > 1/127.0), scale);
    const ChassisSpeeds speeds = {toAngular<LinearVelocity>((rightScaled - leftScaled)/2 * linearMax, this->trackWidth),
                                    (leftScaled + rightScaled)/2 * linearMax};
    moveVel(speeds, this->currentVel);
}

void Chassis::moveVel(ChassisSpeeds speeds,std::pair<LinearVelocity, LinearVelocity> currentVel){
    Number throttle = linearController->getPower(speeds.v.convert(mps),(currentVel.first + currentVel.second).convert(mps) / 2.0);
    Number turn = angularController->getPower(speeds.ω.convert(radps),
                            toAngular<LinearVelocity>(currentVel.first - currentVel.second,this->trackWidth).convert(radps) / 2.0);

    move(throttle - turn, throttle + turn);
}

void Chassis::move(Number left, Number right){
    Number max = units::max(left, right);
    if(max > 1_num){
        left = left/max;
        right = right/max;
    }
    leftMotors.move(left);
    rightMotors.move(right);
}

//dTheta = (dL-dR)/width
//width = (dL-dR)/dTheta
void Chassis::findWidth(){
    this->leftMotors.setAngle(0_stDeg);
    this->rightMotors.setAngle(0_stDeg);

    setPose({0_m,0_m,0_stDeg});
    
    while (true) {
        Controller::updateAll();
        const auto pose = getPose();
        if(Controller::master[RIGHT].pressing){
            move(0.4, -0.4);
        }
        else if(Controller::master[LEFT].pressing){
            move(-0.4, 0.4);
        }
        else {
            move(0,0);
        }
        pros::c::controller_set_text(pros::E_CONTROLLER_MASTER, 2,0,
            std::to_string(
                ((toLinear<Angle>(this->leftMotors.getAngle(),this->wheelDiameter) - 
                toLinear<Angle>(this->rightMotors.getAngle(),this->wheelDiameter))/
                pose.orientation.internal()).convert(in)).c_str());

        if(Controller::master[A].pressed) break;
        pros::delay(50);
    }
}

// s = rTheta
// 2s/Theta = diameter
// 2s/ ((thetaL + thetaR)/2) = diameter
// 4s/(thetaL + thetaR) = diameter
void Chassis::findDiameter(Length distance){
    this->leftMotors.setAngle(0_stDeg);
    this->rightMotors.setAngle(0_stDeg);

    setPose({0_m,0_m,0_stDeg});
    
    while (true) {
        Controller::updateAll();
        const auto pose = getPose();
        if(Controller::master[RIGHT].pressing){
            move(0.4, -0.4);
        }
        else if(Controller::master[LEFT].pressing){
            move(-0.4, 0.4);
        }
        else {
            move(0,0);
        }

        
        pros::c::controller_set_text(pros::E_CONTROLLER_MASTER, 2,0,
            std::to_string(
                ((4*distance.internal())/
                    (toLinear<Angle>(this->leftMotors.getAngle(),this->wheelDiameter) + 
                toLinear<Angle>(this->rightMotors.getAngle(),this->wheelDiameter)).internal())).c_str());

        if(Controller::master[A].pressed) break;
        pros::delay(50);
    }
}
void Chassis::findIMUScalar(Angle rotations){
    this->leftMotors.setAngle(0_stDeg);
    this->rightMotors.setAngle(0_stDeg);

    setPose({0_m,0_m,0_cDeg});
    while (true) {
        Controller::updateAll();
        const auto pose = getPose();
        if(Controller::master[RIGHT].pressing){
            move(0.4, -0.4);
        }
        else if(Controller::master[LEFT].pressing){
            move(-0.4, 0.4);
        }
        else {
            move(0,0);
        }

        
        pros::c::controller_set_text(pros::E_CONTROLLER_MASTER, 2,0,
            std::to_string(double(to_stDeg(rotations)/to_cDeg(pose.orientation)) * this->imu->getGyroScalar().internal()).c_str());
        pros::delay(10);
    }
}

//yoinked from lemlib
void Chassis::calibrateIMU(){
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    auto port = this->imu->getPort();
    while (attempt <= 5) {
        pros::c::imu_reset(port);
        // wait until IMU is calibrated
        do {
            pros::delay(10);
        }
        while (pros::c::imu_get_status(port) & pros::E_IMU_STATUS_CALIBRATING);

        auto heading = pros::c::imu_get_heading(port);
        if (!isnanf(heading) && !isinf(heading)) {
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        std::cout << "failed to calibrate attempt number " << attempt << "\n";
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        this->imu = nullptr;
        std::cout << "failed to calibrate\n";
    }
}
void Chassis::init() {
    if(this->task == nullptr) {
        calibrateIMU();
        this->odom.setPose({0_m,0_m,0_cDeg});
        this->task = new pros::Task([this] {
            std::uint32_t now = pros::millis();
            
            while(pros::c::competition_is_disabled()) pros::Task::delay_until(&now, 5);
            SimpleMovingAverage lSMA(2);
            SimpleMovingAverage rSMA(2);
            auto start = pros::millis();
            while(true) {
                this->mutex.take();

                this->odom.update();
                this->updateVel();
                LinearVelocity target = 0_mps;
                
                switch (getState()) {
                    case ChassisState::IDLE: {
                        break;
                    }
                    case ChassisState::TURN: {
                        units::Pose currentPose = this->odom.getPose();
                        const Angle angularError = getAngularError(this->targetAngle,currentPose.orientation,this->direction);
                        
                        //if passed target or at target, exit
                        if (units::abs(angularError) > 350_stDeg || units::abs(angularError) < 0.2_stDeg) {
                            delete currentProfile;
                            setState(ChassisState::IDLE);
                            move(0_percent,0_percent);
                            this->direction = AUTO;
                            break;
                        }
                        
                        if (this->currentProfile == nullptr) break;
                        const Length d = this->currentProfile->getLength() - this->direction*toLinear<Angle>(
                                    angularError, this->trackWidth);
                        
                        AngularVelocity omega = this->direction*toAngular<LinearVelocity>(
                                                        this->currentProfile->getProfilePoint(d).velocity
                                                       ,this->trackWidth);
                        
                        moveVel({omega,0_mps}, currentVel);
                        break;
                    }
                    case ChassisState::FOLLOW: {
                        units::Pose currentPose = this->odom.getPose();
                        if(this->followReversed) currentPose.orientation = currentPose.orientation + 0.5_stRot;
                        const auto point = this->currentProfile->getProfilePoint(currentPose);

                        this->distTarget = currentProfile->getLength()-point.first.dist;

                        if(point.second >= this->currentProfile->profile.size()-1){
                            setState(ChassisState::IDLE);
                            move(0,0);
                            break;
                        }

                        ChassisSpeeds speeds = {
                            from_radps(point.first.velocity.internal() * point.first.curvature.internal()),
                            point.first.velocity
                        };

                        if (this->followReversed) speeds.v = -speeds.v;

                        if(this->useRAMSETE) speeds = RAMSETE(speeds, point.first.pose, currentPose);
                        moveVel(speeds, this->currentVel);
                        target = speeds.v;
                        break;
                    }
                }

                this->mutex.give();
                pros::Task::delay_until(&now, 10);

                //std::cout <<pros::millis()-start<< "," <<this->currentVel.first.convert(mps)*0.5 + this->currentVel.second.convert(mps)*0.5 
                //        <<","<< target.convert(mps) <<"," <<odom.getPose().y.convert(tile)<< "\n";
                std::cout <<pros::millis()-start<< "," <<
                    toAngular<LinearVelocity>(this->currentVel.first*0.5 - this->currentVel.second*0.5,this->trackWidth).convert(radps) 
                <<"," <<"\n";
            }
            

        },TASK_PRIORITY_MAX-3);
    }
}