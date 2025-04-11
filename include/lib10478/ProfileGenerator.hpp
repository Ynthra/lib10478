#pragma once
#include "Profile.hpp"
#include "bezier.hpp"
#include "lib10478/Odom.hpp"
#include "units/units.hpp"

namespace lib10478 {

struct Constraints {
    VelocityLimits velLimits;
    LinearAcceleration maxAccel;
    LinearAcceleration maxDecel;
    Number frictionCoefficent;
    Length dd = 0.2_cm;
};

enum ProfileType {
    TIME,
    DISTANCE
};

class ProfileGenerator {

private:
    ProfileType profileType;
    Length trackWidth;
    Constraints constraints;
public:
    ProfileGenerator(ProfileType profileType, Constraints constraints, Length trackWidth)
        : constraints(constraints), profileType(profileType), trackWidth(trackWidth) {}
    Profile* generateProfile(const virtualPath& path, std::optional<Constraints> constraints = std::nullopt);
};



} //namespace lib10478