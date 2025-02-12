#include "intake.hpp"
#include "globals.hpp"
#include "lib10478/controller.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include <mutex>

ringColors getColor(){
	if (optical.get_hue() > 190 && optical.get_hue() < 215) return BLUE;
	if (optical.get_hue() > 10 && optical.get_hue() < 35) return RED;
	else return NONE;
}

constexpr double intakeRatio = 68.0/12.0;
#define NEXTTOOTH intakeRatio/3*360
ringColors detectedColor = NONE;
intakeStates intakeState = IDLE;
int timeDescoring = 0;
int timeResetting = 0;
int timeRaising = 0;
Angle intakeTarget = 0_stDeg;
pros::Mutex intakeMutex;

bool shouldIntake = false;
bool shouldReset = false;
void intakeLoop(){
    std::lock_guard<pros::Mutex> lock(intakeMutex);
	const bool isStationary = (fabs(pros::c::motor_get_actual_velocity(intake.getPort())) < 0.01);
	const bool isSettled = (intakeTarget - intake.getAngle()) < 3_stDeg && isStationary;

	if(intakeState == DESCORING) timeDescoring += 10;
	else timeDescoring = 0;
	if(intakeState == RESETTING) timeResetting += 10;
	else timeResetting = 0;
	if(intakeState == RAISING) timeRaising += 10;
	else timeRaising = 0;

    if(intakeState == IDLE){
        if (shouldIntake){
            intake.move(intakeMaxSpeed);
            intakeState = INTAKING;
            shouldIntake = false;
        }
        if(shouldReset){
			intakeTarget = from_stDeg(roundUpToNearestMultiple(to_stDeg(intake.getAngle() - from_stDeg(NEXTTOOTH) * 0.4), NEXTTOOTH,0) + NEXTTOOTH * 0.4);
            pros::c::motor_move_absolute(intake.getPort(),to_stDeg(intakeTarget), 600);
            intakeState = RESETTING;
            shouldReset = false;
        }
    }
	if(timeResetting > 150 && isStationary){
		intake.move(0);
		intakeState = IDLE;
	}

	if(intakeState != DESCORING){
		if(optical.is_installed() && optical.get_proximity() > 240){
			if(detectedColor == NONE) detectedColor = getColor();
			if ((detectedColor != NONE) && (detectedColor != TEAMCOLOR)){
				timeRaising = 0;
				intakeState = RAISING;
				intakeTarget = from_stDeg(roundUpToNearestMultiple(to_stDeg(intake.getAngle()), NEXTTOOTH, NEXTTOOTH * 0.12) + NEXTTOOTH * 0.1);
				pros::c::motor_move_absolute(intake.getPort(),to_stDeg(intakeTarget), 500);
			}
		}
	}
	if(timeRaising > 150 && isStationary){
		pros::c::motor_move_relative(intake.getPort(),-NEXTTOOTH*1.2,600);
		intakeState = DESCORING;
	}
	if(timeDescoring > 150 && isStationary){
		intake.move(0);
		detectedColor = NONE;
		intakeState = IDLE;
	}
}

pros::Task* intakeTask = nullptr;

void intakeInit(){
    if (intakeTask == nullptr) intakeTask = new pros::Task (
        []{
            uint32_t now = pros::millis();
            while(true){
                intakeLoop();
                pros::Task::delay_until(&now, 10);
            }
        }
    );
}

void spinIntake(){
    std::lock_guard<pros::Mutex> lock(intakeMutex);
    shouldIntake = true;
    shouldReset = false;
}
void stopIntake(){
    std::lock_guard<pros::Mutex> lock(intakeMutex);
    shouldIntake = false;
    shouldReset = true;
	if(intakeState == INTAKING) intakeState = IDLE;
}
void outTake(){
	std::lock_guard<pros::Mutex> lock(intakeMutex);
	intakeState = IDLE;
	intake.move(-1);
}


/**
bool outTaking = false;
int timeOutaking = 0;
bool usedIntake = true;
constexpr double intakeRatio = 68.0/12.0;
#define NEXTTOOTH intakeRatio/3*360
bool reversed = false;
ringColors detectedColor = NONE;
std::vector<std::string> outputs;
void intakeControl(controller::Button button){
	usedIntake = (button.releasedTimer < 13000);
	if(outTaking) {timeOutaking += 10; } 
	else{
		if(button.pressing){
			intake.move(100_percent);
		}
		else if (button.releasedTimer > 100 && (pros::c::motor_get_actual_velocity(intake.getPort()) < 0.001)) {
			intake.move(0);
		}
		if(button.released){
			pros::c::motor_move_absolute(intake.getPort(),roundUpToNearestMultiple(to_stDeg(intake.getAngle() - from_stDeg(NEXTTOOTH) * 0.4), NEXTTOOTH,0) + NEXTTOOTH * 0.4, 600);
		}
	}
	if((!reversed) && optical.is_installed() && optical.get_proximity() > 240) {
		outputs.push_back(std::to_string(optical.get_hue()) + "," + std::to_string(optical.get_saturation()));
		if(detectedColor == NONE) detectedColor = getColor();
		if ((detectedColor != NONE) && (detectedColor != TEAMCOLOR)){
			timeOutaking = 0;
			outTaking = true;
			pros::c::motor_move_absolute(intake.getPort(),roundUpToNearestMultiple(to_stDeg(intake.getAngle()), NEXTTOOTH, NEXTTOOTH * 0.12) + NEXTTOOTH * 0.1, 500);
		}	
	}
	
	if((!reversed) && (timeOutaking > 150) && (pros::c::motor_get_actual_velocity(intake.getPort()) < 0.001)){
		pros::c::motor_move_relative(intake.getPort(),-NEXTTOOTH*1.2,600);
		reversed = true;
		timeOutaking = 0;
	}
	if(reversed && (timeOutaking > 300) && (pros::c::motor_get_actual_velocity(intake.getPort()) < 0.001)){
		detectedColor = NONE;
		intake.move(0);
		reversed = false;
		outTaking = false;
		timeOutaking = 0;
	}
	controller::master.set_text(2,0,std::to_string(optical.get_saturation()));
}**/