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
    //why is this shaky??
    chassis.moveToPose(-31.5, 3, 60, 2500,{.forwards=false,.lead=0.85,.maxSpeed=127,.minSpeed=5,.earlyExitRange=5});
    arm.move(1);
    pros::delay(600);
    arm.move(0);
    chassis.waitUntilDone();
    chassis.turnToHeading(60,300);
    chassis.waitUntilDone();
    chassis.tank(-30,-30);
    pros::delay(800);
    clamp.retract();
    pros::delay(150);
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
    chassis.moveToPose(-38, 24, -45 ,2000,{.lead=0.5});
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveToPoint(-32, 17, 1000);
    chassis.waitUntilDone();
    chassis.turnToPoint(4, 57, 1000);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.tank(60, 60);
    pros::delay(800);
    chassis.tank(40, 40);
    pros::delay(200);
    chassis.tank(-30, -30);
    pros::delay(550);
    chassis.tank(30, 30);
    pros::delay(650);
    chassis.tank(-30, -30);
    pros::delay(550);
    chassis.tank(30, 30);
    pros::delay(650);
    chassis.tank(-30, -30);
    pros::delay(950);
    chassis.tank(0, 0);
    pros::delay(100000);
}
void negativeside(){
    
}