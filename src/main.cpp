#include "main.h"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/impl/control/async/asyncMotionProfileControllerBuilder.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

using namespace okapi;

std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
        .withMotors({-2, -4}, {11, 12})
		.withGains(
			{0.001, 0.0001, 0.00001}, // Distance controller gains
			{0.001, 0.0001, 0.00001}, // Turn controller gains
			{0.001, 0, 0}  // Angle controller gains (helps drive straight)
		)
		// Green gearset, 2.75 in wheel diam, 10 in wheel track
        .withDimensions(AbstractMotor::gearset::blue, {{2.75_in, 10.8_in}, int(imev5BlueTPR / 0.75)})
		.withOdometry()
		.buildOdometry();

// std::shared_ptr<AsyncMotionProfileController> profileController =
//   AsyncMotionProfileControllerBuilder()
//     .withLimits({
//       1.0, // Maximum linear velocity of the Chassis in m/s
//       2.0, // Maximum linear acceleration of the Chassis in m/s/s
//       10.0 // Maximum linear jerk of the Chassis in m/s/s/s
//     })
//     .withOutput(chassis)
//     .buildMotionProfileController();

// Motor intake(-20);
pros::IMU imu(15);

double constrainAngle(double angle) {
	while (angle < -180) {
		angle += 360;
	}
	while (angle > 180) {
		angle -= 360;
	}
	return angle;
}

void initialize() {
	imu.reset();
	uint32_t calibrationStart = pros::millis();
	pros::lcd::initialize();
	// profileController->generatePath({
	// 	{0_ft, 0_ft, 0_deg},
	// 	{3_ft, 1_ft, 90_deg}},
	// 	"A");
	pros::Task::delay_until(&calibrationStart, 2000);
}

void disabled() {}
void competition_initialize() {}
void autonomous() {
	// profileController->setTarget("A");
	// profileController->waitUntilSettled();
	chassis->moveDistance(3_ft);
}

void opcontrol() {
	Controller primary;
	ControllerButton runAuto(ControllerDigital::A);
	// ControllerButton runOuttake(ControllerDigital::R1);
	// ControllerButton runIntake(ControllerDigital::R2);

	while (true) {
		chassis->getModel()->curvature(
				primary.getAnalog(ControllerAnalog::leftY),
				primary.getAnalog(ControllerAnalog::rightX));
		if (runAuto.changedToPressed()) {
			autonomous();
		}
		// if (runIntake.isPressed()) {
		// 	intake.controllerSet(1);
		// } else if (runOuttake.isPressed()) {
		// 	intake.controllerSet(-1);
		// } else {
		// 	intake.controllerSet(0);
		// }
		pros::delay(20);
	}
}