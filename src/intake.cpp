#include "intake.hpp"
#include "globals.hpp"
#include "hardware/Motor/Motor.hpp"
#include "pros/misc.h"

Colour getColour(){
    if ((!colorSensor.is_installed()) || colorSensor.get_proximity() < 250 || colorSensor.get_saturation() < 0.3) return NONE;
    if (colorSensor.get_hue() > 200 && colorSensor.get_hue() < 220) return BLUE;
    if (colorSensor.get_hue() > 5 && colorSensor.get_hue() < 25) return RED;
    return NONE;
}


Angle targetAngle = 0_stDeg;

int timeSorting = 0;
int timeStopped = 0;
bool isStopped = false;


void detectRing(){
    Colour detectedColour = getColour();
    if (detectedColour != NONE && detectedColour != teamColour){
        isSorting = true;
        targetAngle = SETPOINT(0.89);
        pros::c::motor_move_absolute(11,to_stDeg(targetAngle), 600);
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
    }
}
void sort(){
    const bool pastTarget = topIntake.getAngle() > (targetAngle - 0.5_stDeg);
    if (!isSorting){
        timeSorting = 0;
        return;
    }
    timeSorting += 10;

    if(!isStopped) {
        if(pastTarget && timeSorting > 50) {
            isStopped = true;
            timeStopped = 0;
            intake.move(0);
        }
    }
    else {
        timeStopped += 10;
        if(timeStopped > 50){
            isSorting = false;
            isStopped = false;
            timeSorting = 0;
            timeStopped = 0;
        }
    }
}