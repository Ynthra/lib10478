#include "Chassis.hpp"
#include "Profile.hpp"
#include "controller.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Port.hpp"
#include "lib10478/Odom.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "pros/imu.h"
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
                 TrackingWheel* backTracker)
    : leftMotors(leftPorts,outputVelocity), rightMotors(rightPorts,outputVelocity), 
      swappedSides(swappedSides),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      leftTracker(&leftMotors,wheelDiameter,-trackWidth/2),
      rightTracker(&rightMotors,wheelDiameter,trackWidth/2),
      imu(imu),
      odom(imu, &leftTracker, &rightTracker, backTracker), 
      linearController(linearController),angularController(angularController) {}

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

std::pair<AngularVelocity, AngularVelocity> Chassis::toMotorSpeeds(ChassisSpeeds speeds)
{
	const LinearVelocity velLeft = speeds.v - toLinear<AngularVelocity>(speeds.ω, this->trackWidth);
	const LinearVelocity velRight = speeds.v + toLinear<AngularVelocity>(speeds.ω, this->trackWidth);

    return std::make_pair(toAngular<LinearVelocity>(velLeft,wheelDiameter),
                          toAngular<LinearVelocity>(velRight,wheelDiameter));
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
    const Length distance = toLinear<Angle>(getAngularError(angle, pose.orientation, direction),this->trackWidth);
    //this->direction = turnDirection(sgn(distance.internal()));
    this->targetAngle = angle;
    //currentProfile = this->generateProfile(CubicBezier({0_m,0_m},{0_m,0.33*distance},{0_m,0.66*distance},{0_m,distance}));
    setState(ChassisState::TURN);
}

void Chassis::waitUntilDist(Length d){
    auto now = pros::millis();
    while(true){
        std::lock_guard<pros::Mutex> lock(mutex);
        if(distTarget < d) break;
        pros::Task::delay_until(&now, 10);
    }
}

void Chassis::CancelMovement()  
{
    std::lock_guard<pros::Mutex> lock(mutex);
    setState(ChassisState::IDLE);
    //tank(0_percent,0_percent);
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
    auto pose = odom.getPose();
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
std::pair<LinearVelocity, LinearVelocity> Chassis::getVel(){
    const Time now = from_msec(pros::millis());
    const Time dt =  now - this->timestamp;

    const Angle leftAngle = leftMotors.getAngle();
    const Angle rightAngle = rightMotors.getAngle();

    const LinearVelocity leftVel = toLinear<AngularVelocity>((leftAngle-this->leftPrev)/dt,this->wheelDiameter);
    const LinearVelocity rightVel = toLinear<AngularVelocity>((rightAngle-this->rightPrev)/dt,this->wheelDiameter);

    this->leftPrev = leftAngle;
    this->rightPrev = rightAngle;

    if (leftAngle != leftPrev || rightAngle != rightPrev || dt > 50_msec) this->timestamp = now;

    return {leftVel, rightVel};
}   

void Chassis::tank(AngularVelocity maxVel, double scale){
    const LinearVelocity linearMax = toLinear<AngularVelocity>(maxVel,this->wheelDiameter);
    const double left = Controller::master[LEFT_Y];
    const double right = Controller::master[RIGHT_Y];
    const double leftScaled = driveCurve(left * (fabs(left) > 1/127.0), scale);
    const double rightScaled = driveCurve(right * (fabs(right) > 1/127.0), scale);
    const auto vel = getVel();
    const ChassisSpeeds speeds = {toAngular<LinearVelocity>((rightScaled - leftScaled)/2 * linearMax, this->trackWidth),
                                    (leftScaled + rightScaled)/2 * linearMax};
    moveVel(speeds, vel.first, vel.second);
}

void Chassis::moveVel(ChassisSpeeds speeds,LinearVelocity leftVel, LinearVelocity rightVel){
    Number throttle = linearController->getPower(speeds.v.convert(mps),(leftVel + rightVel).convert(mps) / 2.0);
    Number turn = angularController->getPower(speeds.ω.convert(radps),
                            toAngular<LinearVelocity>(rightVel - leftVel,this->trackWidth).convert(radps) / 2.0);

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
void Chassis::findWidth(Angle rotations){
    this->leftMotors.setAngle(0_stDeg);
    this->rightMotors.setAngle(0_stDeg);

    while (true) {
        /**controller::update();
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
        pros::delay(50);**/
    }
}

// s = rTheta
// 2s/Theta = diameter

void Chassis::findDiameter(Length distance){
    this->leftMotors.setAngle(0_stDeg);
    this->rightMotors.setAngle(0_stDeg);
    while (true) {
        /**controller::update();
        controller::master.set_text(2,0,std::to_string(2*distance.convert(in)/
        ((this->leftMotors.getAngle().internal() + this->rightMotors.getAngle().internal())/2)));
        if(controller::A.pressed) break;
        pros::delay(50);**/
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
        do pros::delay(10);
        while (pros::c::imu_get_status(port) != (pros::E_IMU_STATUS_ERROR || pros::E_IMU_STATUS_CALIBRATING));
        // exit if imu has been calibrated
        auto heading = pros::c::imu_get_heading(port);
        if (!isnanf(heading) && !isinf(heading)) {
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        //lemlib::infoSink()->warn("IMU failed to calibrate! Attempt #{}", attempt);
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        this->imu = nullptr;
        //lemlib::infoSink()->error("IMU calibration failed, defaulting to tracking wheels / motor encoders");
    }
}
void Chassis::init() {
    if(this->task == nullptr) {
        calibrateIMU();
        this->odom.setPose({0_m,0_m,0_cDeg});
        this->task = new pros::Task([this] {
            std::uint32_t now = pros::millis();
            
            while(true) {
                this->mutex.take();
                this->odom.update();
                
                switch (getState()) {
                    case ChassisState::IDLE:
                        break;
                    case ChassisState::TURN:
                        break;
                    case ChassisState::FOLLOW:
                        break;
                }

                this->mutex.give();
                pros::Task::delay_until(&now, 10);
            }
            

        },TASK_PRIORITY_DEFAULT+2);
    }
    /**if (task == nullptr) {
        imu->calibrate();
        while(imu->isCalibrating()){
            pros::delay(10);
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
                if (motorSpeeds.first != 0_rpm ) ouputs.push_back(std::to_string(time) + "," + std::to_string(currentVelocity.first.convert(rpm)) + "," + std::to_string(motorSpeeds.first.convert(rpm)));
                switch (getState()) {
                
                case ChassisState::IDLE:
                    motorSpeeds = {0_rpm,0_rpm};
                    break;
                
                case ChassisState::FOLLOW: {
                    units::Pose currentPose = odom.getPose();
                    if (followReversed) currentPose.orientation = currentPose.orientation + from_stDeg(M_PI);
                    const ProfilePoint point = currentProfile->getProfilePoint(currentPose);
                    distTarget = currentProfile->getLength()-point.dist;

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
                    const Angle angularError = getAngularError(this->targetAngle,currentPose.orientation,this->direction);
                    
                    if (units::abs(angularError) > 350_stDeg || units::abs(angularError) < 0.2_stDeg) {
                        delete currentProfile;
                        setState(ChassisState::IDLE);
                        tank(0_percent,0_percent);
                        this->direction = AUTO;
                        break;
                    }

                    const Length d = currentProfile->getLength() - this->direction*toLinear<Angle>(
                                angularError, this->trackWidth);
                    
                    if (currentProfile == nullptr) break;
                    LinearVelocity velocity = currentProfile->getProfilePoint(d).velocity;


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
    }**/
}