#pragma once

#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include <utility>
#include <vector>


namespace lib10478{

class VelocityLimits {
public: 
    VelocityLimits(LinearVelocity defaultVel): defaultVel(defaultVel) {}
    VelocityLimits& from(Length pos) {
        currentStart = pos;
        return *this;
    }
    VelocityLimits& to(Length pos) {
        currentEnd = pos;
        return *this;
    }
    VelocityLimits& set(LinearVelocity vel) {
        zones.emplace_back(currentStart, currentEnd, vel);
        return *this;
    }
    LinearVelocity at(Length pos) {
        for (const auto& zone: zones) {
            if(pos >= zone.start && pos <= zone.end){
                return zone.maxVel;
            }
        }
        return defaultVel;
    }
private:
    struct VelocityZone {
        Length start;
        Length end;
        LinearVelocity maxVel;
        VelocityZone(Length start, Length end, LinearVelocity vel) 
        : start(start), end(end), maxVel(vel) {}
    };
    std::vector<VelocityZone> zones;
    Length currentStart = 0_m;
    Length currentEnd = 0_m;
    LinearVelocity defaultVel;
};

struct ProfilePoint{
    units::Pose pose;
    LinearVelocity velocity;
    Curvature curvature;
    Length dist;

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
    std::pair<ProfilePoint,int> getProfilePoint(units::V2Position pos);
    Length getLength();
    
    const Length dd;
    std::vector<ProfilePoint> profile;

    int prev = 0;
};



}//namespace lib10478