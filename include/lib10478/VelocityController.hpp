#pragma once
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace lib10478 {

class VelocityController{
    private:
        double kV, kAU, kAD, kS, kP;
        mutable AngularVelocity prevTargetVel = 0_radps;
    public:

        VelocityController(double kV, double kAU, double kAD, double kS, double kP = 0);

        Number getPower(AngularVelocity velocity, AngularVelocity currentVelocity) const;
};

}// namespace lib10478