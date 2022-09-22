#include "main.h"

#include "typhoon/filter.hpp"

void initialize() {}
void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
	filter::PositionFilter pf{StateVector(0, 0, 0, 0, 0),
	                          StateVector(1, 1, 1, 1, 1).asDiagonal(),
	                          OutputVector(1, 1, 1, 1).asDiagonal()};
}
