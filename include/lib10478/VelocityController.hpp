#pragma once
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace lib10478 {

class VelocityController{
    private:
        double kV, kAU, kAD, kS, kP;
        mutable double prevTargetVel = 0;
        mutable double prevVel = 0;
        mutable uint32_t timestamp = 0;
    public:

        VelocityController(double kV, double kAU, double kAD, double kS, double kP = 0);

        Number getPower(double velocity, double currentVelocity) const;
};

}// namespace lib10478