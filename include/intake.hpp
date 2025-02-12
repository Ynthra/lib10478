#pragma once
#include "units/units.hpp"

enum ringColors{
	RED,
	BLUE,
	NONE
};

enum intakeStates{
	IDLE,
	RESETTING,
	RAISING,
	INTAKING,
	DESCORING
};

inline Number intakeMaxSpeed = 1;
extern ringColors getColor();
extern void spinIntake();
extern void stopIntake();
extern void intakeInit(); 
extern void outTake();