#pragma once
#include "lib10478/controller.hpp"
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
extern void intakeLoop(bool spin);
extern void waitUntilStored(int timeout = 0);