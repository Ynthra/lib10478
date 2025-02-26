#include "auton.hpp"
#include "intake.hpp"
#include "lib10478/Chassis.hpp"
#include "lib10478/Profile.hpp"
#include "lib10478/bezier.hpp"
#include "lib10478/controller.hpp"
#include "lib10478/lib10478.hpp"
#include "globals.hpp"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <atomic>
#include <string>
#include <sys/_intsup.h>
constexpr int x = (TEAMCOLOR == BLUE)-(TEAMCOLOR == RED);

void trueSoloWP(){
//30 deg measuring tool: lib10478::CubicBezier({17.071797_in, -20.000000_in}, {21.274170_in, -22.426757_in}, {21.279994_in, -22.430049_in}, {24.000000_in, -24.000000_in});
    auto start = pros::millis();
    chassis.setPose({55.5_in, -30.797_in,90_cDeg});

    auto getGoal = chassis.generateProfile(
        lib10478::CubicBezier({55.500000_in, -30.797000_in}, {40.000000_in, -30.797000_in}, {43.026660_in, -32.307406_in}, {31.601750_in, -25.877330_in})
    );
    chassis.followProfile(getGoal,{.followReversed=true});

    auto getFirstRing = chassis.generateProfile(
        lib10478::CubicBezier({31.601750_in, -25.877330_in}, {44.899275_in, -33.347200_in}, {32.099995_in, -38.563995_in}, {23.144591_in, -43.073010_in})
    );
    chassis.waitUntilSettled(); delete getGoal;
    pros::delay(100);
    clamp.retract(); //clamp goal
    pros::delay(50);
    chassis.followProfile(getFirstRing);
    pros::delay(100);
    intakeLoop(true);

    auto getSecondRing = chassis.generateProfile(
        lib10478::CubicBezier({23.144591_in, -43.073010_in}, {43.810859_in, -22.312341_in}, {74.988679_in, -18.566038_in}, {33.147170_in, 10.415094_in})
        ,0.2_cm,{{lib10478::VelocityLimits{maxVel}.from(30_in).to(100_in).set(maxVel*0.6), maxAccel*0.5, maxDeccel*0.8, 0.15}});
    chassis.waitUntilSettled(); delete getFirstRing;
    pros::delay(200);
    chassis.turnTo(chassis.getPose().angleTo({43.810859_in, -22.312341_in}));
    chassis.waitUntilSettled();

    chassis.followProfile(getSecondRing);
    pros::delay(200);
    intakeLoop(false);
    clamp.extend(); //drop goal
    pros::delay(100);
    //intakePiston.extend(); //lift intake
    bool gotRing = false;
    std::atomic<bool> stillRunning = true;
    pros::Task intakeTask ([&]() {
        gotRing = waitUntilStored(5500,true);
        stillRunning = false;
    });

    //intakePiston.retract(); //drop intake

    chassis.waitUntilSettled(); delete getSecondRing;
    
    clamp.retract(); //drop clamp (to help allign)
    pros::delay(50);
    {
        const units::Pose pose = chassis.getPose();
        const Length dist = (pose.y+3_in)/units::sin(pose.orientation);
        chassis.driveStraight(-dist,{.followReversed=(dist > 0_m)});
        chassis.waitUntilSettled();
    }
    //chassis.driveStraight(6_in);
    //chassis.waitUntilSettled();
    {
        const units::Pose pose = chassis.getPose();
        const Length dist = (pose.y-0.5_in)/units::sin(pose.orientation);
        chassis.driveStraight(-dist,{.followReversed=(dist > 0_m)});
        chassis.waitUntilSettled();
    }
    chassis.turnTo(-90_cDeg);
    chassis.waitUntilSettled();
    while(stillRunning) pros::delay(10);
    
    chassis.tank(-1, -1);
    pros::delay(250);
    chassis.tank(0, 0);
    pros::delay(100);
    {
        const units::Pose pose = chassis.getPose();
        chassis.setPose({pose.x,0_m,pose.orientation});
    }
    pros::delay(50);
    chassis.driveStraight(1.0_in);
    chassis.waitUntilSettled();
    auto now = pros::millis();

    auto getThirdRing = chassis.generateProfile(
        lib10478::CubicBezier({61.800000_in, 0.000000_in}, {46.273300_in, 0.000000_in}, {65.569811_in, 39.939623_in}, {19.562264_in, 40.845283_in})
    );
    pros::Task::delay_until(&now, 300);
    if(gotRing) {
        intakeLoop(true);
        pros::delay(250);
        intakeLoop(false);
    }
    controller::master.set_text(2,0,std::to_string(chassis.getPose().x.convert(in)));
    chassis.followProfile(getThirdRing);
    pros::delay(200);
    intakeLoop(true);
    auto toLadder = chassis.generateProfile(
        lib10478::Spline({
            new lib10478::CubicBezier({19.562264_in, 40.845283_in}, {19.534862_in, 31.011143_in}, {23.390970_in, 32.405906_in}, {23.473015_in, 26.088452_in}),
            new lib10478::CubicBezier({23.473015_in, 26.088452_in}, {23.555060_in, 19.770999_in}, {20.986697_in, 16.980962_in}, {16.253068_in, 12.715141_in})
        })
    );
    chassis.waitUntilSettled(); delete getThirdRing;
    waitUntilStored(4000);
    chassis.turnTo(chassis.getPose().angleTo({19.534862_in, 31.011143_in})+180_stDeg);
    chassis.waitUntilSettled();
    clamp.extend(); //lift clamp to prep for goal

    chassis.followProfile(toLadder,{.followReversed=true});
    chassis.waitUntilDist(24_in);
    clamp.retract(); //clamp goal
    chassis.waitUntilSettled();
    /**chassis.driveStraight(-0.8_tile,{.followReversed = true});
    chassis.waitUntilSettled();
    clamp.retract(); //clamp goal**/

    //chassis.setPose({61.8_in,0_in,chassis.getPose().orientation});
}

void skills(){
    chassis.setPose({-59.9_in,0_in,90_cDeg});
    std::atomic<lbTargets> lbtarget = DOWN;
    pros::Task lbTask ([&]() {
        while(true){
            const double kG = -1; //accounts for weight of arm

            const Angle armAngle = armSensor.getAngle()-360_stDeg;
            const double error = to_stDeg(from_stDeg(lbtarget.load())-armAngle);

            //if(error < 1 && lbtarget == DOWN) arm.move(0);
            arm.move(kG*sin(to_stDeg(armAngle)) + armPID.update(error));
            pros::delay(10);
        }
    });
    auto now = pros::millis();
    intakeLoop(true);
    pros::Task::delay_until(&now,300);
    intakeLoop(false);
    chassis.driveStraight(13_in);
    chassis.waitUntilSettled();
    chassis.turnTo(0_cDeg);
    chassis.waitUntilSettled();
    chassis.driveStraight(-19.6_in,{.followReversed=true});
    auto collectFour = chassis.generateProfile(
        lib10478::Spline({
            new lib10478::CubicBezier({-46.900000_in, -20.000000_in}, {-24.982885_in, -17.910524_in}, {-19.889792_in, -23.869029_in}, {-18.555788_in, -33.719350_in}),
            new lib10478::CubicBezier({-18.555788_in, -33.719350_in}, {-17.221785_in, -43.569671_in}, {-22.607038_in, -45.153568_in}, {-32.427205_in, -46.420687_in}),
            new lib10478::CubicBezier({-32.427205_in, -46.420687_in}, {-42.247372_in, -47.687805_in}, {-49.374913_in, -49.588483_in}, {-56.660843_in, -54.340176_in})
        })
        ,0.2_cm, {{maxVel*0.9, maxAccel*0.25, maxDeccel*0.5, 0.1}}
    );
    chassis.waitUntilSettled();
    clamp.retract(); // clamp first goal
    pros::delay(50);
    chassis.turnTo(chassis.getPose().angleTo({-24.982885_in, -17.910524_in}));
    chassis.waitUntilSettled();
    chassis.followProfile(collectFour);
    intakeLoop(true);

    auto toFirstWallRing = chassis.generateProfile(
        lib10478::CubicBezier({-51.640000_in, -60.090000_in}, {-35.175780_in, -50.701631_in}, {-32.773918_in, -58.896500_in}, {1.441409_in, -64.076510_in})
    );

    chassis.waitUntilDist(2_in);
    chassis.CancelMovement();
    chassis.turnTo(chassis.getPose().angleTo({-46.840676_in,-58.458311_in}),lib10478::CW);
    chassis.waitUntilSettled();
    chassis.driveStraight(17_in);
    chassis.waitUntilSettled();
    chassis.turnTo(chassis.getPose().angleTo({-72_in,-72_in})+ 180_stDeg);
    chassis.waitUntilSettled();
    chassis.driveStraight(-6_in,{.followReversed=true});
    chassis.waitUntilSettled();
    pros::delay(50);
    clamp.extend();
    pros::delay(20);
    chassis.followProfile(toFirstWallRing);

    pros::delay(200);
    bool gotRing = false;
    std::atomic<bool> stillRunning = true;
    pros::Task intakeTask ([&]() {
        gotRing = waitUntilStored(3000, true);
        stillRunning = false;
    });
    lbtarget.store(ALLIGNED);
    chassis.waitUntilSettled(); delete toFirstWallRing;
    {
        const units::Pose pose = chassis.getPose();
        const Length dist = (pose.x)/units::cos(pose.orientation);
        chassis.driveStraight(-dist,{.followReversed=(dist > 0_m)});
        chassis.waitUntilSettled();
    }
    chassis.turnTo(180_cDeg);
    chassis.waitUntilSettled();
    while(stillRunning) pros::delay(10);
    intakeLoop(true);
    chassis.driveStraight(2_in);
    chassis.waitUntilSettled();
    chassis.tank(0.2,0.2);
    pros::delay(100);
    intakeLoop(false);
    intake.move(0);
    pros::delay(50);
    lbtarget.store(SCORING);
    pros::delay(650);
    lbtarget.store(DOWN);
    pros::delay(200);
    chassis.driveStraight(-4_in,{.followReversed = true});
    chassis.waitUntilSettled();
    chassis.turnTo(chassis.getPose().angleTo({-46.622901_in,0_in}) + 180_stDeg);
    chassis.waitUntilSettled();
    {
        auto pose = chassis.getPose();
        chassis.driveStraight(-pose.distanceTo(
            {units::Vector2D<Length>{-46.622901_in,0_in} -
            0.5_in * units::Vector2D<Number>::fromPolar(pose.orientation + 180_stDeg,1)}),
        {.followReversed=true});
    }
    chassis.waitUntilSettled();
    chassis.turnTo(180_cDeg);
    chassis.waitUntilSettled();
    pros::delay(50);
    chassis.setPose({-46.9_in,0_in,chassis.getPose().orientation});


    //repeat here
    chassis.driveStraight(-19.6_in,{.followReversed=true});
    auto collectFour2nd = chassis.generateProfile(
        lib10478::Spline({
            new lib10478::CubicBezier({-46.900000_in, 20.000000_in}, {-24.982885_in, 17.910524_in}, {-19.889792_in, 23.869029_in}, {-18.555788_in, 33.719350_in}),
            new lib10478::CubicBezier({-18.555788_in, 33.719350_in}, {-17.221785_in, 43.569671_in}, {-22.607038_in, 45.153568_in}, {-32.427205_in, 46.420687_in}),
            new lib10478::CubicBezier({-32.427205_in, 46.420687_in}, {-42.247372_in, 47.687805_in}, {-49.374913_in, 49.588483_in}, {-56.660843_in, 54.340176_in})
        })
        ,0.2_cm, {{maxVel*0.9, maxAccel*0.25, maxDeccel*0.5, 0.1}}
    );
    chassis.waitUntilSettled();
    clamp.retract(); // clamp second goal
    pros::delay(50);
    chassis.turnTo(chassis.getPose().angleTo({-24.982885_in, 17.910524_in}));
    chassis.waitUntilSettled();
    chassis.followProfile(collectFour2nd);
    intakeLoop(true);
    auto toSecondWallRing = chassis.generateProfile(
        lib10478::CubicBezier({-51.640000_in, 60.090000_in}, {-35.175780_in, 50.701631_in}, {-32.773918_in, 58.896500_in}, {1.441409_in, 64.076510_in})
    );

    chassis.waitUntilDist(2_in);
    chassis.CancelMovement();
    chassis.turnTo(chassis.getPose().angleTo({-46.840676_in,58.458311_in}),lib10478::CW);
    chassis.waitUntilSettled();
    chassis.driveStraight(14_in);
    chassis.waitUntilSettled();
    chassis.turnTo(chassis.getPose().angleTo({-72_in,72_in})+ 180_stDeg);
    chassis.driveStraight(-6_in,{.followReversed=true});
    chassis.waitUntilSettled();

    auto pose = chassis.getPose();
    controller::master.clear();
    pros::delay(50);
    controller::master.set_text(1,0,std::to_string(pose.x.convert(in)));
    pros::delay(50);
    controller::master.set_text(2,0,std::to_string(pose.y.convert(in)));
}