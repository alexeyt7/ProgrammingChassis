#include "main.h"

#include "typhoon/model.hpp"

void initialize() {}
void disabled() {}
void competition_initialize() {}
void autonomous() {}

auto rad = [](double deg) { return deg * M_PI / 180; };
auto deg = [](double rad) { return rad / M_PI * 180; };

void opcontrol() {
	// filter::PositionFilter pf{StateVector(0, 0, 0, 0, 0),
	//                           StateVector(1, 1, 1, 1, 1).asDiagonal(),
	//   OutputVector(1, 1, 1, 1).asDiagonal()};
	StateVector x0 = {0, 0, 0, 0, 0};
	InputVector u = {1, -1};
	StateVector x1 = modelFunction(x0, u, 1, 1);
	std::cout << "x0:\n" << x0 << std::endl;
	std::cout << "u:\n" << u << std::endl;
	std::cout << "x1:\n" << x1 << std::endl;
}
