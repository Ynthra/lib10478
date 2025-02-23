#include "auton.hpp"
#include "intake.hpp"
#include "lib10478/bezier.hpp"
#include "lib10478/controller.hpp"
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
    //spinIntake();
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
    //stopIntake();
    pros::delay(50);
    intakeMaxSpeed = 1;
    //spinIntake();
    chassis.waitUntilSettled();
    intakePiston.retract(); //drop it
    int time = 0;
    bool found = true;
    while (optical.get_proximity() < 240 || getColor() != TEAMCOLOR) {
        //spinIntake();
        pros::delay(10);
        time += 10;
        if(time > 1000) {
            found = false;
            break;
        }
    }
    //stopIntake();
    pros::delay(300);
    chassis.followProfile(toWallStake,{.useRAMSETE=true,.followReversed=true});
    chassis.waitUntilSettled();
    pros::delay(100);
    //if(found) spinIntake();
    pros::delay(500);
    //stopIntake();
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

void trueSoloWP(){
    auto now = pros::millis();
    chassis.setPose({55.5_in, -30.797_in,90_cDeg});
    auto getGoal = chassis.generateProfile(
        lib10478::CubicBezier({55.500000_in, -30.797000_in}, {40.000000_in, -30.797000_in}, {41.728972_in, -31.699957_in}, {32.307587_in, -26.622341_in})
    );
    chassis.followProfile(getGoal,{.followReversed=true});
    auto getFirstRing = chassis.generateProfile(
        lib10478::CubicBezier({32.307587_in, -26.622341_in}, {42.344851_in, -32.028937_in}, {33.250025_in, -37.094680_in}, {24.294621_in, -41.603695_in})
    );
    chassis.waitUntilSettled(); delete getGoal;
    pros::delay(100);
    clamp.retract(); //clamp goal
    pros::delay(100);
    chassis.followProfile(getFirstRing);
    pros::delay(100);
    intakeLoop(true);
    auto getSecondRing = chassis.generateProfile(
        lib10478::CubicBezier({24.294621_in, -41.603695_in}, {41.377923_in, -20.770709_in}, {54.489798_in, -17.399726_in}, {41.689198_in, -0.382562_in})
    );
    chassis.waitUntilSettled(); delete getFirstRing;
    pros::delay(200);
    chassis.turnTo(45_cDeg);
    chassis.waitUntilSettled();
    intakeLoop(false);
    clamp.extend(); //drop goal
    chassis.followProfile(getSecondRing);
    auto driveBack = chassis.generateProfile(
        lib10478::CubicBezier({45.131435_in, -4.653162_in}, {45.821841_in, -5.571819_in}, {45.685227_in, -5.298148_in}, {46.113366_in, -5.293836_in})
    );
    pros::delay(300);
    intakePiston.extend(); //lift intake
    bool gotRing = false;
    pros::Task intakeTask ([&]() {
        gotRing = waitUntilStored(3000);
    });
    chassis.waitUntilSettled(); delete getSecondRing;
    std::cout << (pros::millis()-now) << std::endl;
    intakePiston.retract(); //drop intake
    pros::delay(200);
    clamp.retract(); //drop clamp (to help allign)
    pros::delay(50);
    chassis.driveStraight(-2.5_in,{.followReversed=true});
    chassis.waitUntilSettled();
    chassis.turnTo(-90_cDeg);
    chassis.waitUntilSettled();

    pros::delay(1000);
    if(!gotRing) controller::master.rumble(",");
    chassis.tank(-0.5, -0.5);
    pros::delay(300);
    chassis.tank(-0.3, -0.3);
    pros::delay(450);
    chassis.tank(0, 0);
    pros::delay(150);
    chassis.driveStraight(1.0_in);
    chassis.waitUntilSettled();
    pros::delay(300);
    if(gotRing) intakeLoop(true);
    pros::delay(250);
    intakeLoop(false);
    //chassis.setPose({59_in,0_in,chassis.getPose().orientation});
   
}