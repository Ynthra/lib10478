#include "main.h"
#include "auton.hpp"
#include "intake.hpp"
#include "lib10478/Math.hpp"
#include "lib10478/bezier.hpp"
#include "lib10478/lib10478.hpp"
#include "lib10478/controller.hpp"
#include "globals.hpp"
#include "pros/device.hpp"
#include "units/Angle.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include <cmath>
#include <string>
#include <sys/_intsup.h>
#include <vector>

void initialize() 
{ 
	optical.set_led_pwm(100);
	optical.set_integration_time(20);

	chassis.init();
	//intakeInit();
	pros::Task screenTask([&]() {
	SimpleMovingAverage intakePowSMA(20);
	while (true) {
		const units::Pose pose = chassis.getPose();
		pros::screen::print(pros::E_TEXT_MEDIUM,0,("x: " + std::to_string(pose.x.convert(in))).c_str());
		pros::screen::print(pros::E_TEXT_MEDIUM,1,("y: " + std::to_string(pose.y.convert(in))).c_str());
		pros::screen::print(pros::E_TEXT_MEDIUM,2,("angle: " + std::to_string(to_cDeg(pose.orientation))).c_str());
		pros::screen::print(pros::E_TEXT_MEDIUM,3,("intake power: " + std::to_string(intakePowSMA.next(pros::c::motor_get_power(intake.getPort())))).c_str());
	

		pros::delay(50);
	}
});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	const auto testauton = chassis.generateProfile(
		lib10478::Spline({new lib10478::CubicBezier({57.954106_in, 24.270531_in}, {33.799517_in, 24.753623_in}, {39.596618_in, 35.623188_in}, {23.896135_in, 35.623188_in}),
						  new lib10478::CubicBezier({23.896135_in, 35.623188_in}, {8.195652_in, 35.623188_in}, {15.200483_in, 46.975845_in}, {-0.500000_in, 46.975845_in})
						})
	);
	chassis.setPose({57.954106_in, 24.270531_in,-90_cDeg});
	chassis.followProfile(testauton);
	chassis.waitUntilSettled();
	pros::delay(100);
	/**skillsBack();
	controller::master.set_text(0,0,"finished auto");
	pros::delay(5000);**/
}

bool usedIntake = false;
void intakeControl(controller::Button button){
	usedIntake = button.releasedTimer < 13000;
	if(button.pressed && intakePiston.is_extended()){
		intakePiston.retract();
	}
	intakeLoop(button.pressing);
}

enum lbTargets{
	DOWN = -119,
	ALLIGNED = -77,
	RAISED = -60,
	SCORING = 52
};
lbTargets lbtarget = DOWN;

void armControl(controller::Button button){
	if (button.pressed && lbtarget == DOWN) {
		lbtarget = ALLIGNED;
	}
	if(button.released && lbtarget == ALLIGNED){
		lbtarget = RAISED;
	}
	if(button.pressed && lbtarget == RAISED){
		lbtarget = SCORING;
	}
	if(button.released && lbtarget == SCORING){
		lbtarget = DOWN;
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
		doinker.toggle();
	}
}
void intakePistonControl(controller::Button button){
	if(button.pressed){
		intakePiston.toggle();
	}
}

enum Drivers{
	DARIUS,
	ADE
};

bool startedDriver = false;
int timer;
bool past45s = false;
bool exitCorner = false;
void opcontrol()
{
	pros::c::motor_set_encoder_units(intake.getPort(), pros::E_MOTOR_ENCODER_DEGREES);
	pros::c::motor_set_gearing(intake.getPort(), pros::E_MOTOR_GEAR_600);
	chassis.CancelMovement();
	Drivers driver = DARIUS;

	if(startedDriver == false){
		timer = pros::millis();
		startedDriver = true;

	}
	//waitUntilStored(20000);
	while (true) {
		controller::update();
		if(controller::X.pressed){
			for(std::string out: ouputs){
				std::cout << out << std::endl;
			}

		}
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
		//controller::master.set_text(2,0,std::to_string(to_cDeg(chassis.getPose().orientation)));
		chassis.tank(controller::driveCurve(controller::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
						1, 8, 127, 0.1, 1), 
					controller::driveCurve(controller::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 
						1, 8, 127, 0.1, 1));
		
		if (driver == ADE) {
			intakeControl(controller::L1);
			armControl(controller::L2);
			clampControl(controller::R1);
			doinkerControl(controller::R2);
			intakePistonControl(controller::Up);
		}
		if(driver == DARIUS){
			intakeControl(controller::L1);
			armControl(controller::R2);
			clampControl(controller::R1);
			doinkerControl(controller::L2);
			intakePistonControl(controller::Up);
		}
		pros::delay(10);
	}
}