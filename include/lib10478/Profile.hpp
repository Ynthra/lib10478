#pragma once

#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include <vector>

namespace lib10478{

struct ProfilePoint{
    units::Pose pose;
    LinearVelocity velocity;
    Curvature curvature;

    bool operator==(ProfilePoint other) const {
        return pose.x == other.pose.x && pose.y == other.pose.y 
            && pose.orientation == other.pose.orientation
            && velocity == other.velocity
            && curvature == other.curvature;
    }
};

class Profile
{
public:
    Profile(std::vector<ProfilePoint>& profile, Length dd);
    ProfilePoint getProfilePoint(Length d);
    ProfilePoint getProfilePoint(units::V2Position pos);
    Length getLength();
    
    const Length dd;
    std::vector<ProfilePoint> profile;

    int prev = 0;
};


}//namespace lib10478