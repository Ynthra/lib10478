#include "main.h"
#include "auton.hpp"
#include "intake.hpp"
#include "lib10478/Chassis.hpp"
#include "lib10478/Math.hpp"
#include "lib10478/bezier.hpp"
#include "lib10478/lib10478.hpp"
#include "lib10478/controller.hpp"
#include "globals.hpp"
#include "pros/device.hpp"
#include "pros/misc.hpp"
#include "units/Angle.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include <cmath>
#include <iostream>
#include <iterator>
#include <string>
#include <sys/_intsup.h>
#include <vector>


std::vector<std::string> outputs;
void initialize() 
{ 
	optical.set_led_pwm(100);
	optical.set_integration_time(20);

	pros::delay(150);

	pros::c::motor_set_encoder_units(intake.getPort(), pros::E_MOTOR_ENCODER_DEGREES);
	pros::c::motor_set_gearing(intake.getPort(), pros::E_MOTOR_GEAR_600);

	horizontalWheel.reset();
	chassis.init();

	pros::delay(100);
	chassis.setPose({0_m,0_m,0_cDeg});
	pros::Task screenTask([&]() {
	SimpleMovingAverage intakePowSMA(20);
	{
		//const units::Pose pose = chassis.getPose();
		//std::cout <<pose.x.convert(in) << "," << pose.y.convert(in) << "\n";
	}
	units::Pose pose = chassis.getPose();
	while (true) {
		auto prevPose = pose;
		pose = chassis.getPose();
		pros::screen::print(pros::E_TEXT_MEDIUM,0,("x: " + std::to_string(pose.x.convert(in))).c_str());
		pros::screen::print(pros::E_TEXT_MEDIUM,1,("y: " + std::to_string(pose.y.convert(in))).c_str());
		pros::screen::print(pros::E_TEXT_MEDIUM,2,("angle: " + std::to_string(to_cDeg(pose.orientation))).c_str());
		pros::screen::print(pros::E_TEXT_MEDIUM,3,("intake power: " + std::to_string(intakePowSMA.next(pros::c::motor_get_power(intake.getPort())))).c_str());
		
		if(pros::competition::is_autonomous() && (pose.x != prevPose.x) && (pose.y != prevPose.y)){
			ouputs.push_back(
			std::to_string(pose.x.convert(in)) + "," +
			std::to_string(pose.y.convert(in))
			);
		}
		//std::cout <<pose.x.convert(in) << "," << pose.y.convert(in) << "\n";
		//std::cout <<"(" <<pose.x.convert(in) << "," << pose.y.convert(in) << "),";

		//pros::screen::print(pros::E_TEXT_MEDIUM,5,("back dist: " + std::to_string(horizontalWheel.getDistance().convert(in))).c_str());
		/**pros::screen::print(pros::E_TEXT_MEDIUM,1,("left dist: " + std::to_string(chassis.leftTracker.getDistance().convert(in))).c_str());
		pros::screen::print(pros::E_TEXT_MEDIUM,2,("right dist: " + std::to_string(chassis.rightTracker.getDistance().convert(in))).c_str());
		**/
		pros::delay(50);
	}
	});
}

void disabled() {}

void competition_initialize() {}
//15.5 inch height
//13.5 inch width
void autonomous() {
	skills();
	//trueSoloWP();
}

bool usedIntake = false;
void intakeControl(controller::Button button){
	usedIntake = button.releasedTimer < 13000;
	if(button.pressed && intakePiston.is_extended()){
		intakePiston.retract();
	}
	intakeLoop(button.pressing);
}


void armControl(controller::Button mainButton, controller::Button hangButton){
	if (mainButton.pressed && lbtarget == DOWN) {
		lbtarget = ALLIGNED; 
	}
	if(mainButton.released && lbtarget == ALLIGNED){
		intake.move(0);
		lbtarget = RAISED;
	}
	if(mainButton.pressed && lbtarget == RAISED){
		lbtarget = SCORING;
	}
	if(mainButton.released && lbtarget == SCORING){
		lbtarget = DOWN;
	}

	if(hangButton.pressed){
		lbtarget = HANGING;
	}

	if (mainButton.pressed && lbtarget == HANGING) {
		lbtarget = SCORING; 
	}
	const double kG = -1; //accounts for weight of arm

	const Angle armAngle = armSensor.getAngle()-360_stDeg;
	const double error = to_stDeg(from_stDeg(lbtarget)-armAngle);

	//if(error < 1 && lbtarget == DOWN) arm.move(0);
	arm.move(kG*sin(to_stDeg(armAngle)) + armPID.update(error));
	//controller::master.set_text(2,0,std::to_string(to_stDeg(from_stDeg(lbtarget)-armSensor.getAngle()+360_stDeg)));
}

void clampControl(controller::Button button){
	if(button.pressed){
		clamp.toggle();
	}
}
void doinkerControl(controller::Button button){
	if(button.pressed){
		ldoinker.toggle();
	}
}
void intakePistonControl(controller::Button button){
	if(button.pressed){
		intakePiston.toggle();
	}
}

enum Controls{
	MAIN,
	SKILLS
};

bool startedDriver = false;
int timer;
bool past45s = false;
bool exitCorner = false;
void opcontrol()
{
	std::cout << "hi \n";
	chassis.CancelMovement();
	Controls control = SKILLS;

	if(startedDriver == false){
		timer = pros::millis();
		startedDriver = true;

	}
	while (true) {
		controller::update();
		if(!past45s && (pros::millis()-timer)>(60000+45000-45000)){
				past45s = true;
				controller::master.rumble("...");
		}
		if(!exitCorner && (pros::millis()-timer)>(60000+45000-32000)){
				exitCorner = true;
				controller::master.rumble("...");
				//release goal if in corner
				if(usedIntake == false){
					clamp.extend();
				}
		}
		if(controller::A.pressed){
			const units::Pose pose = chassis.getPose();
			const Length dist = pose.y*units::sin(pose.orientation);
			chassis.driveStraight(-dist,{.followReversed=(dist > 0_m)});
		}
		if(controller::Y.pressed){
			for (auto out: outputs){
				std::cout << out << std::endl;
			}
		}
		//controller::master.set_text(2,0,std::to_string(to_cDeg(chassis.getPose().orientation)));
		chassis.tank(controller::driveCurve(controller::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
						1, 8, 127, 0.1, 1), 
					controller::driveCurve(controller::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 
						1, 8, 127, 0.1, 1));
		
		if(control == MAIN){
			intakeControl(controller::L1);
			armControl(controller::R2,controller::X);
			clampControl(controller::R1);
			doinkerControl(controller::L2);
			intakePistonControl(controller::Up);
		}
		else if(control == SKILLS) {
			intakeControl(controller::L1);
			armControl(controller::R2,controller::X);
			clampControl(controller::R1);
			if(controller::L2.pressed){
				chassis.driveStraight(1_in);
				chassis.waitUntilSettled();
				pros::delay(300);
				intakeLoop(true);
				pros::delay(250);
			}
			intakePistonControl(controller::Up);
		}
		pros::delay(10);
	}
}