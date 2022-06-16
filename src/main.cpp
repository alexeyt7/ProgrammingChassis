#include "main.h"

using namespace okapi;

std::shared_ptr<ChassisController> drive =
    ChassisControllerBuilder()
        .withMotors({-2, -4}, {11, 12})
        .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
        .build();

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
	Controller primary;

	while (true) {
		drive->getModel()->curvature(
				primary.getAnalog(ControllerAnalog::leftY),
				primary.getAnalog(ControllerAnalog::rightX));
		pros::delay(20);
	}
}