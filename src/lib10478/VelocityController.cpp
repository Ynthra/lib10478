#include "lib10478/VelocityController.hpp"
#include "units/Angle.hpp"
#include "lib10478/Math.hpp"

using namespace lib10478;

VelocityController::VelocityController(double kV, double kAU, double kAD, double kS, double kP)
    : kV(kV), kAU(kAU), kAD(kAD), kS(kS), kP(kP) {}

Number VelocityController::getPower(AngularVelocity velocity, AngularVelocity currentVelocity) const {
    const AngularAcceleration accel = (velocity - prevTargetVel) / (10_msec);
    const Number power = kV * velocity.convert(rpm) + kS*sgn(velocity.internal()) 
                        + (kAU * (accel.convert(rpm2) > 0) + kAD * (accel.convert(rpm2) < 0)) * accel.convert(rpm2)
                        + kP * (velocity - currentVelocity).convert(rpm); 

    prevTargetVel = velocity;
    return power;
}