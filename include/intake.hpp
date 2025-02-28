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

inline bool sortingEnabled = true;
inline Number intakeMaxSpeed = 1;
extern ringColors getColor();
extern void intakeLoop(bool spin);
extern bool waitUntilStored(int timeout = 0, bool down = false);