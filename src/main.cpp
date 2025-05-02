#include "main.h"
#include "autons.hpp"
#include "globals.hpp"
#include "hardware/IMU/IMU.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Motor/Motor.hpp"
#include "intake.hpp"
#include "lib10478/Chassis.hpp"
#include "lib10478/Math.hpp"
#include "lib10478/bezier.hpp"
#include "lib10478/controller.hpp"
#include "pros/device.hpp"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <system_error>

void initialize() {
	pros::delay(100);
	colorSensor.set_integration_time(20);
	pros::delay(100);
	colorSensor.set_led_pwm(100);
	pros::c::motor_set_encoder_units(11, pros::E_MOTOR_ENCODER_DEGREES);
	pros::c::motor_set_gearing(11, pros::E_MOTOR_GEAR_BLUE);
	chassis.calibrate();
	pros::delay(100);
	pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen

            pros::screen::print(pros::E_TEXT_MEDIUM,0, "X: %f", chassis.getPose().x); // x
            pros::screen::print(pros::E_TEXT_MEDIUM,1, "Y: %f", chassis.getPose().y); // y
            pros::screen::print(pros::E_TEXT_MEDIUM,2, "Theta: %f", chassis.getPose().theta); // heading
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	//sevenRingsafe();
	chassis.moveToPoint(0,24,2000);
	chassis.waitUntilDone();
	chassis.moveToPoint(0,48,2000);
	chassis.waitUntilDone();
	chassis.moveToPoint(0,36,2000,{.forwards=false});
	chassis.waitUntilDone();
	chassis.moveToPoint(0,24,2000,{.forwards=false});
	chassis.waitUntilDone();
	chassis.moveToPoint(0,16,2000,{.forwards=false});
	chassis.waitUntilDone();
	chassis.moveToPoint(0,8,2000,{.forwards=false});
	chassis.waitUntilDone();
}

double offset = 0.5;
int loops = 0;
void armControl(){
    bool pressed = Controller::master[R2].pressed;
    bool released = Controller::master[R2].released;
    
	if (pressed && lbtarget == IDLE) {
		lbtarget = ALLIGNED; 
	}
	if(released && lbtarget == ALLIGNED){
		lbtarget = RAISED;
	}
	if(pressed && lbtarget == RAISED){
		lbtarget = SCORING;
	}
	if(released && lbtarget == SCORING){
		lbtarget = IDLE;
	}

	if(Controller::master[X].pressed){
		lbtarget = HANGING;
	}

	if (pressed && lbtarget == HANGING) {
		lbtarget = SCORING; 
	}
	const double kG = -0.01; //accounts for weight of arm

	Angle armAngle = 360_stDeg - armSensor.getAngle();

	Angle target = from_stDeg(lbtarget);

	if(lbtarget == ALLIGNED) target += from_stDeg(offset);
	const double error = to_stDeg(target-armAngle);

	if(fabs(error) < 6 && lbtarget == IDLE) arm.move(0);
	else arm.move(kG*sin(to_stDeg(armAngle)) + armPID.update(error));
	//pros::c::controller_set_text(pros::E_CONTROLLER_MASTER,2,0,std::to_string(to_stDeg(armAngle)).c_str());
	/**if(controller::Left.pressed){
		offset += 0.5;
	}
	if(controller::Right.pressed){
		offset -= 0.5;
	}**/
	//controller::master.set_text(2,0,std::to_string(int(ALLIGNED) + offset));
}

bool startedDriver = false;
bool past45s = false;
bool exitCorner = false;
uint32_t timer = pros::millis();
void opcontrol()
{
    if(startedDriver == false){
        timer = pros::millis();
        startedDriver = true;
    }
    while (true) {
        Controller::updateAll();

        if(!past45s && (pros::millis()-timer)>(60000+45000-45000)){
            past45s = true;
            pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "...");
        }
        if(!exitCorner && (pros::millis()-timer)>(60000+45000-32000)){
            exitCorner = true;
            pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "...");
        }
        
        //chassis.tank(550_rpm);
        
		//std::cout << (pros::millis()-timer) << "," << colorSensor.get_proximity()<< "," << colorSensor.get_hue() << "," << colorSensor.get_saturation() << "\n";
		
		detectRing();
		if(isSorting){
			sort();
		}
		if(!isSorting){
			if(Controller::master[L1].pressing){
				intake.move(100_percent);
				if(intakePiston.is_extended()){
					intakePiston.retract();
				}
			}
			else if(Controller::master[DOWN].pressing) {
				intake.move(-100_percent);
			}
			if(Controller::master[L1].released){
				pros::c::motor_move_absolute(11,to_stDeg(SETPOINT(0.65)), 400);
				pros::c::motor_move(12, 0);
			}
		}
		if(Controller::master[R1].pressed){
			clamp.toggle();
		}
		if(Controller::master[L2].pressed){
			lDoinker.toggle();
		}
        if(Controller::master[UP].pressed){
            intakePiston.toggle();
        }
        armControl();
        pros::delay(10);
    }
}