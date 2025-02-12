#pragma once
#include "hardware/IMU/Imu.hpp"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/pose.hpp"
#include "hardware/encoder/Encoder.hpp"


namespace lib10478 {

class TrackingWheel
{
    public:
        TrackingWheel(lemlib::Encoder* encoder, Length diameter, Length offset, Number ratio = 1);
        Length getDistance();
        Length getDistanceDelta();
        Length getOffset();
        int reset();
    private:
        lemlib::Encoder* m_encoder;
        Length m_diameter;
        Length m_offset;
        Number m_ratio;
        Length m_lastTotal;
};

class Odom
{
    public:
        Odom(lemlib::IMU* imu, TrackingWheel* left, TrackingWheel* right, TrackingWheel* back = nullptr);
        void update();
        units::Pose getPose();
        void setPose(units::Pose pose);
    private:
        Angle m_offset = 0_stRad;
        pros::Mutex mutex;
        lemlib::IMU* m_imu;
        TrackingWheel* m_left;
        TrackingWheel* m_right;
        TrackingWheel* m_back;
        units::Pose m_pose = {0_m,0_m,0_cDeg};
};

} // namespace lib10478