#include "autons.hpp"
#include "globals.hpp"
#include "intake.hpp"
#include "pros/rtos.hpp"

void sevenRingsafe(){
    chassis.setPose({0,0,148.5});
    arm.move(-1);
    pros::delay(600);
    arm.move(0);
    chassis.tank(-80, -80);
    pros::delay(200);
    chassis.moveToPose(-31.5, 3, 60, 2500,{.forwards=false,.lead=0.75,.maxSpeed=90});
    arm.move(1);
    pros::delay(600);
    arm.move(0);
    chassis.waitUntilDone();
    pros::delay(200);
    clamp.retract();
    pros::delay(200);
    pros::Task task([&](){
        while (true) {
            detectRing();
            if(isSorting){
                sort();
            }
            if(!isSorting){
                intake.move(1);
            }
        }
    });
    chassis.moveToPose(-35 , 24, -45 ,2000,{.lead=0.35});
    chassis.waitUntilDone();
}