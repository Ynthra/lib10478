#include "auton.hpp"
#include "intake.hpp"
#include "lib10478/bezier.hpp"
#include "lib10478/lib10478.hpp"
#include "globals.hpp"
#include "pros/device.hpp"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include <sys/_intsup.h>
constexpr int x = (TEAMCOLOR == BLUE)-(TEAMCOLOR == RED);
void solowp(){
    const auto getGoal = chassis.generateProfile(lib10478::CubicBezier({56.436_in*x, -30.293_in}, {50.106_in*x, -30.293_in},{42.251_in*x, -29.355_in},{31.231_in*x, -22.321_in}),0.1_in);

    chassis.setPose({56.436_in*x, -30.293_in,from_cDeg(90*x)});

    chassis.followProfile(getGoal,{.useRAMSETE= true,.followReversed = true});
    const auto getFirstRing = chassis.generateProfile(lib10478::CubicBezier({31.348_in*x, -23.376_in}, {38.441_in*x, -27.904_in}, {35.334_in*x, -36.038_in}, {24.549_in*x, -39.086_in}),0.1_in);
    const auto getSecondRing = chassis.generateProfile(lib10478::CubicBezier({24.549_in*x, -39.086_in}, {50.574_in*x, -21.032_in}, {52.685_in*x, -12.356_in}, {40.961_in*x, -2.9777_in}),0.1_in);
    const auto toWallStake = chassis.generateProfile(lib10478::CubicBezier({40.961_in*x, -2.9777_in}, {57.960_in*x, -16.576_in}, {58.429_in*x, 0.53927_in}, {67.339_in*x, 0.53927_in}),0.1_in);


    chassis.waitUntilSettled();
    clamp.retract(); //collect goal
    pros::delay(10);
    intakeMaxSpeed = 0.8;
    spinIntake();
    chassis.followProfile(getFirstRing,{.useRAMSETE=true});
    chassis.waitUntilSettled();
    pros::delay(300);
    chassis.tank(x*1, x*-1);
    pros::delay(450);
    chassis.tank(0, 0);
    chassis.followProfile(getSecondRing,{.useRAMSETE=true});
    intakePiston.extend(); //lift intake
    pros::delay(200);
    clamp.extend(); //drop goal
    stopIntake();
    pros::delay(50);
    intakeMaxSpeed = 1;
    spinIntake();
    chassis.waitUntilSettled();
    intakePiston.retract(); //drop it
    int time = 0;
    bool found = true;
    while (optical.get_proximity() < 240 || getColor() != TEAMCOLOR) {
        spinIntake();
        pros::delay(10);
        time += 10;
        if(time > 1000) {
            found = false;
            break;
        }
    }
    stopIntake();
    pros::delay(300);
    chassis.followProfile(toWallStake,{.useRAMSETE=true,.followReversed=true});
    chassis.waitUntilSettled();
    pros::delay(100);
    if(found) spinIntake();
    pros::delay(500);
    stopIntake();
    chassis.tank(1,1);
    pros::delay(400);
    chassis.tank(0, 0);

}

void skillsBack(){
    chassis.tank(-1, -1);
    pros::delay(300);
    clamp.toggle();
    pros::delay(100);
    chassis.tank(0, 0);
    pros::delay(100);
    intake.move(1);
    pros::delay(300);
    chassis.tank(-0.8, -0.8);
    pros::delay(500);
    chassis.tank(0, 0);
    clamp.toggle();
    chassis.tank(-0.8, -0.8);
}