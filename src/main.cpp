#include "main.h"
#include "lib10478/controller.hpp"
#include <cmath>

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol()
{

    auto now = pros::millis();
    while (true) {
        Controller::updateAll();

        std::cout << pros::millis() - now<< "," << Controller::master()[LEFT_Y] << "\n";
        pros::delay(5);
    }
}