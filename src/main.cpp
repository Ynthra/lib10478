#include "main.h"
#include "hardware/IMU/IMU.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "lib10478/Chassis.hpp"
#include "lib10478/controller.hpp"
#include <cmath>

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

        chassis.tank(Controller::master[LEFT_Y],Controller::master[RIGHT_Y]);
        
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