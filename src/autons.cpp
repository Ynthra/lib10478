#include "autons.hpp"
#include "globals.hpp"
#include "intake.hpp"
#include "pros/rtos.hpp"
void negativeside(){
    // Flipped initial pose (if needed, adjust heading)
    chassis.setPose({0, 0, -148.5}); // Mirror heading if necessary
    
    // Arm sequence (unchanged)
    //pros::delay(2500);
    arm.move(-1);
    pros::delay(600);
    arm.move(0);
    
    // Initial backward drive (unchanged)
    chassis.tank(-80, -80);
    pros::delay(200);
    
    // Mirror X-coordinates and angles for left side
    chassis.moveToPose(31.5, 3, -60, 2500, {.forwards=false, .lead=0.85, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5});
    arm.move(1);
    pros::delay(600);
    
    // Second move (same mirrored target)
    chassis.moveToPose(31.5, 3, -60, 2500, {.forwards=false, .lead=0.85, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5});
    arm.move(1);
    pros::delay(600);
    arm.move(0);
    chassis.waitUntilDone();
    
    // Turn direction flipped
    chassis.turnToHeading(-60, 300); // Negative angle for left side
    chassis.waitUntilDone();
    
    // Backward push (unchanged)
    chassis.tank(-30, -30);
    pros::delay(800);
    clamp.retract();
    pros::delay(150);
    
    // Intake task (unchanged)
    pros::Task task([&](){
        while (pros::competition::is_autonomous()) {
            detectRing();
            if(isSorting){
                sort();
            }
            if(!isSorting){
                intake.move(1);
            }
        }
    });
    
    // Mirrored movement sequence
    chassis.moveToPose(38, 24, 45, 2000, {.lead=0.5}); // X and angle flipped
    chassis.waitUntilDone();
    pros::delay(200);
    
    chassis.moveToPoint(32, 17, 1000); // X flipped
    chassis.waitUntilDone();
    
    chassis.turnToPoint(-4, 44, 1000); // X flipped
    chassis.waitUntilDone();
    pros::delay(100);
    
    // Tank movements (unchanged, no coordinates)
    chassis.tank(60, 60);
    pros::delay(800);
    chassis.tank(45, 45);
    pros::delay(200);
    chassis.tank(-80, -80);

    /*
    pros::delay(550);
    chassis.tank(30, 30);
    pros::delay(650);
    chassis.tank(-30, -30);
    pros::delay(550);
    chassis.tank(30, 30);
    pros::delay(650);
    chassis.tank(-30, -30);
    pros::delay(550);
    chassis.tank(30,30);
    pros::delay(650);
    chassis.tank(-30,-30);
    pros::delay(950);
    chassis.tank(0, 0);
    pros::delay(100000);
*/
}
void sevenRingsafe(){
    chassis.setPose({0,0,148.5});
    //pros::delay(2000);
    arm.move(-1);
    pros::delay(600);
    arm.move(0);
    chassis.tank(-80, -80);
    pros::delay(200);
    chassis.moveToPose(-31.5, 3, 60, 2500,{.forwards=false,.lead=0.85,.maxSpeed=127,.minSpeed=5,.earlyExitRange=5});
    arm.move(1);
    pros::delay(600);
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
        while (pros::competition::is_autonomous()) {
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
    pros::delay(600);
    arm.move(-83);
    pros::delay(200);
    chassis.moveToPose(-31.5, 3, 60, 2500,{.forwards=false,.lead=0.85,.maxSpeed=127,.minSpeed=5,.earlyExitRange=5});
    chassis.tank(0,0);
    pros::delay(100000);
    /*
    pros::delay(550);
    chassis.tank(30, 30);
    pros::delay(650);
    chassis.tank(-30, -30);
    pros::delay(550);
    chassis.tank(30, 30);
    pros::delay(650);
    chassis.tank(-30, -30);
    pros::delay(550);
    chassis.tank(30,30);
    pros::delay(650);
    chassis.tank(-90,-90);
    pros::delay(200);
    arm.move(-0.50);
    chassis.turnToHeading(60,300);
    pros::delay(950);
    chassis.tank(0, 0);
    pros::delay(100000);
    */
}
