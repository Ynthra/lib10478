#include "intake.hpp"
#include "globals.hpp"
#include "lib10478/controller.hpp"
#include "pros/device.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <mutex>

ringColors getColor(){
	if (optical.get_hue() > 200 && optical.get_hue() < 220) return BLUE;
	if (optical.get_hue() > 0 && optical.get_hue() < 20) return RED;
	else return NONE;
}

constexpr double intakeRatio = 68.0/12.0;
#define NEXTTOOTH intakeRatio/3*360
ringColors detectedColor = NONE;

bool sorting = false;
int timeSorting = 0;
bool stopped = false;
Angle intakeAngle = 0_stDeg;
bool storeRing = false;
bool prevSpin = false;
int settledTime = 0;
int stoppedTime = 0;
void intakeLoop(bool spin){
	const bool isStationary = pros::c::motor_get_actual_velocity(intake.getPort()) < 0.001;
	const bool atTarget = (intakeAngle - intake.getAngle()) < 5_stDeg;
	const bool pastTarget = intake.getAngle() > intakeAngle;
	if (atTarget) settledTime += 10;
	else settledTime = 0;
	if(isStationary) stoppedTime+=10;
	else stoppedTime = 0;
	if((detectedColor == NONE) && optical.is_installed() && optical.get_proximity() > 240) {
		detectedColor = getColor();
		if ((detectedColor != NONE) && (detectedColor != TEAMCOLOR)){
			sorting = true;
			intakeAngle = from_stDeg(roundUpToNearestMultiple(to_stDeg(intake.getAngle()- from_stDeg(NEXTTOOTH) * 0.69), NEXTTOOTH, 0) + NEXTTOOTH * 0.69);
			intake.move(100_percent);
			stoppedTime = 0;
		}	
	}
	if(sorting) {
		timeSorting += 10;
		if (!stopped){
			if (pastTarget && timeSorting > 50) {
				timeSorting = 0;
				stopped = true;
				intake.move(0);
			}
			return;
		}
		else {
			if(timeSorting > 200){
				detectedColor = NONE;
				stopped = false;
				timeSorting = 0;
				sorting = false;
				intakeAngle = from_stDeg(roundUpToNearestMultiple(to_stDeg(intake.getAngle() - from_stDeg(NEXTTOOTH) * 0.4), NEXTTOOTH,0) + NEXTTOOTH * 0.4);
				pros::c::motor_move_absolute(intake.getPort(),to_stDeg(intakeAngle), 600);
				stoppedTime = 0;
				if(storeRing) {
					intake.move(100_percent);
					return;
				}
			}
			else {
				return;
			}
		}
	}
	if (storeRing) {
		if (detectedColor == TEAMCOLOR) {
			intake.move(0);
			storeRing = false;
			detectedColor = NONE;	
		}
		return;
	}
	else{
		detectedColor = NONE;
	}
	if(spin){
		intake.move(100_percent);
		stoppedTime = 0;
	}
	if(!spin && prevSpin){
		if(lbtarget == ALLIGNED){
			intakeAngle = from_stDeg(roundUpToNearestMultiple(to_stDeg(intake.getAngle() - from_stDeg(NEXTTOOTH) * 0.73), NEXTTOOTH,0) + NEXTTOOTH * 0.73);
		}
		else{
			intakeAngle = from_stDeg(roundUpToNearestMultiple(to_stDeg(intake.getAngle() - from_stDeg(NEXTTOOTH) * 0.4), NEXTTOOTH,0) + NEXTTOOTH * 0.4);
		}
		pros::c::motor_move_absolute(intake.getPort(),to_stDeg(intakeAngle), 600);
		stoppedTime = 0;
	}
	if (!spin && ((settledTime > 200) && isStationary) || (stoppedTime > 300)) {
		intake.move(0);
	}
	prevSpin = spin;
}

bool waitUntilStored(int timeout){
	if (timeout == 0) timeout = 60000;
	storeRing = true;
	auto now = pros::millis();
	intake.move(70_percent);
	while (storeRing) {
		if ((pros::millis() - now) > timeout) {
			storeRing = false;
			intakeLoop(false);
			return false;
		}
		intakeLoop(true);
		pros::delay(10);
	}
	intakeLoop(false);
	pros::delay(500);
	pros::c::motor_move_relative(intake.getPort(),-0.35*NEXTTOOTH,300);
	pros::delay(500);
	return true;
} 