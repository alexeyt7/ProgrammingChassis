#include "main.h"

using namespace okapi;

std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
        .withMotors({-2, -4}, {11, 12})
		.withGains(
			{0.001, 0.00015, 0.00001}, // Distance controller gains
			{0.001, 0.00015, 0.00001}, // Turn controller gains
			{0.001, 0, 0}  // Angle controller gains (helps drive straight)
		)
		.withClosedLoopControllerTimeUtil(10, 1, 250_ms)
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

double constrainAngle(double angle) {
	while (angle < -180) {
		angle += 360;
	}
	while (angle > 180) {
		angle -= 360;
	}
	return angle;
}

pros::IMU imu(15);

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
	QAngle turns[] = {45_deg, 90_deg, 180_deg};
	double error[3];

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 3; j++) {
			double initial = imu.get_heading();
			chassis->turnAngle(turns[j]);
			error[j] = constrainAngle((imu.get_heading() - initial) - turns[j].convert(degree));
		}
		pros::lcd::print(i, "error: %.4f %.4f %.4f", error[0], error[1], error[2]);
	}
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