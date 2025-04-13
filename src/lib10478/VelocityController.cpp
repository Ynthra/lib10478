#include "lib10478/VelocityController.hpp"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "lib10478/Math.hpp"

using namespace lib10478;

VelocityController::VelocityController(double kV, double kAU, double kAD, double kS, double kP)
    : kV(kV), kAU(kAU), kAD(kAD), kS(kS), kP(kP) {}

Number VelocityController::getPower(double velocity, double currentVelocity) const {
    const double dt = (pros::millis() - timestamp) / 1000.0;
    const double accel = (velocity - prevTargetVel) / dt;
    const Number power = kV * velocity + kS*sgn(velocity) 
                        + (kAU * (accel > 0) + kAD * (accel < 0)) * accel
                        + kP * (velocity - currentVelocity);

    if (currentVelocity != prevVel) timestamp = pros::millis();
    prevTargetVel = velocity;
    prevVel = currentVelocity;
    return power;
}