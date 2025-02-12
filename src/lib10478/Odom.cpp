#include "lib10478/Odom.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/pose.hpp"
#include "units/units.hpp"
#include <mutex>

using namespace lib10478;

TrackingWheel::TrackingWheel(lemlib::Encoder* encoder, Length diameter, Length offset, Number ratio)
    : m_encoder(encoder), 
      m_diameter(diameter), 
      m_offset(offset), 
      m_ratio(ratio), 
      m_lastTotal(0_m) {}

Length TrackingWheel::getDistance(){
    return toLinear<Angle>(m_encoder->getAngle(),m_diameter) * m_ratio;
}
Length TrackingWheel::getDistanceDelta(){
    const Length total = getDistance();
    const Length delta = total - m_lastTotal;
    m_lastTotal = total;
    return delta;
}
Length TrackingWheel::getOffset() { return m_offset; }

int TrackingWheel::reset() { return m_encoder->setAngle(0_stRad); }


Odom::Odom(lemlib::IMU* imu, TrackingWheel* left, TrackingWheel* right, TrackingWheel* back)
    : m_imu(imu), m_left(left), m_right(right), m_back(back) {}

units::Pose Odom::getPose()
{
    std::lock_guard<pros::Mutex> lock (mutex);
    return m_pose; 
}

void Odom::setPose(units::Pose pose)
{ 
    std::lock_guard<pros::Mutex> lock (mutex);
    m_offset += pose.orientation - m_pose.orientation;
    m_pose = pose; 
}

void Odom::update()
{
    std::lock_guard<pros::Mutex> lock (mutex);
    Angle angle = 0_stRad;
    if (m_imu->isConnected())
        angle = -m_imu->getRotation() + 180_stDeg;
    else{
        angle = from_stRad((m_left->getDistance().internal() - m_right->getDistance().internal() ) / 
                            (m_left->getOffset().internal() - m_right->getOffset().internal())) + 90_stDeg;
    }
    const Angle Theta = angle + m_offset;
    const Angle dTheta = Theta - m_pose.orientation;    

    const units::V2Position lateralDeltas = {(m_left->getDistanceDelta() + m_right->getDistanceDelta())*0.5,
                                              m_back ? m_back->getDistanceDelta() : 0_m };
    const units::V2Position lateralOffsets = {(m_left->getOffset() + m_right->getOffset()) * 0.5,
                                              m_back ? m_back->getOffset() : 0_m};

    const units::V2Position localOffset = [&] {
        if (dTheta == 0_stRad) return lateralDeltas; // prevent divide by 0                                        
        return 2 * units::sin(dTheta / 2) * (lateralDeltas / to_stRad(dTheta) + lateralOffsets);
    }();
    
    m_pose += localOffset.rotatedBy(m_pose.orientation + dTheta / 2);
    m_pose.orientation = Theta;
}