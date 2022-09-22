#include "main.h"

#include "typhoon/filter.hpp"

void initialize() {}
void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
	filter::PositionFilter pf;
}
