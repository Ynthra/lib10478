#include "main.h"
#include "hardware/IMU/IMU.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Motor/Motor.hpp"
#include "lib10478/Chassis.hpp"
#include "lib10478/controller.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <cmath>
#include <system_error>

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol()
{
    lemlib::V5InertialSensor imu(7);

    lib10478::Chassis chassis(
        {1,2,3},{-4,-5,-6}, //drive ports
        false,
        &imu,
        450_rpm, //drive speed
        12.5_in, //drive width
        3.25_in, //wheel diameter
        nullptr,nullptr, //no velocity controllers
        nullptr //no tracking wheels
    );

    while (true) {
        Controller::updateAll();

        

        //chassis.tank(550_rpm);
        
        if(Controller::master[L1].pressing){
            //spin intake
        }
        else {
            //stop intake
        }

        if(Controller::master[R1].pressed){
            //toggle clamp
        }

        pros::delay(10);
    }
}