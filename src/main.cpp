#include "main.h"
#include "autons.hpp"
#include "globals.hpp"
#include "hardware/Motor/Motor.hpp"
#include "intake.hpp"
#include "lib10478/controller.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <cmath>
#include <cstdint>

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
	negativeside();
	
}


void armControl(){
	static double offset = 0.5;
	static int loops = 0;
	static bool control = true;

	if(Controller::master[UP].pressed){
		arm.move(-1);
		control = false;
	}
	else if(Controller::master[DOWN].pressed){
		arm.move(1);
		control = false;
	}
	if(Controller::master[UP].released || Controller::master[DOWN].released){
		arm.move(0);
	}
	
    bool pressed = Controller::master[R2].pressed;
    bool released = Controller::master[R2].released;
	if (pressed || released){
		control = true;
	}
    
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

	if(control){
		if(fabs(error) < 10 && lbtarget == IDLE) arm.move(0);
		else arm.move(kG*sin(to_stDeg(armAngle)) + armPID.update(error));
	}
	
	//pros::c::controller_set_text(pros::E_CONTROLLER_MASTER,2,0,std::to_string(to_stDeg(armAngle)).c_str());
	/**if(controller::Left.pressed){
		offset += 0.5;
	}
	if(controller::Right.pressed){
		offset -= 0.5;
	}**/
	//controller::master.set_text(2,0,std::to_string(int(ALLIGNED) + offset));
}
inline float driveCurve(float input, float deadzone, float scale, float maxjoy, float minvolt, float maxvolt) {
	if(fabs(input)>deadzone){
		return (input *
	(powf(2.718, -(scale / (0.1* maxjoy))) + powf(2.718, (fabs(input) - maxjoy) / (0.1*maxjoy)) * (1 - powf(2.718, -(scale / (0.1*maxjoy))))) 
	*(1-(minvolt/maxvolt)) * (maxvolt/maxjoy) + minvolt * (input<0?-1:1));
	}
	else{
		return 0;
	}
}

bool startedDriver = false;
bool past45s = false;
bool exitCorner = false;
uint32_t timer = pros::millis();
extern int timeStationary;
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
        
		chassis.tank(127*driveCurve(Controller::master[LEFT_Y]*127, 
						1, 8, 127, 0.04, 1), 
					127*driveCurve(Controller::master[RIGHT_Y]*127, 
						1, 8, 127, 0.04, 1));
        
		//std::cout << (pros::millis()-timer) << "," << colorSensor.get_proximity()<< "," << colorSensor.get_hue() << "," << colorSensor.get_saturation() << "\n";
		
		if(topIntake.getActualVelocity() < 0.1_radps) timeStationary += 10;
		else timeStationary = 0;
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
				if(timeStationary > 50) timeStationary = 50;
			}
			else if(Controller::master[B].pressing) {
				intake.move(-100_percent);
			}
			if(Controller::master[L1].released){
				if (lbtarget != ALLIGNED) pros::c::motor_move_absolute(11,to_stDeg(SETPOINT(0.65)), 400);
				else pros::c::motor_move(11, 0);
				pros::c::motor_move(12, 0);
			}
			if(Controller::master[B].released){
				pros::c::motor_move(12, 0);
				pros::c::motor_move(11, 0);
			}
		}
		if(Controller::master[R1].pressed){
			clamp.toggle();
		}
		if(Controller::master[L2].pressed){
			lDoinker.toggle();
		}
        if(Controller::master[X].pressed){
            intakePiston.toggle();
        }
        armControl();
        pros::delay(10);
    }
}