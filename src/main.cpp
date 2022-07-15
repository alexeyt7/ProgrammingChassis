#include "main.h"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/chassis/model/readOnlyChassisModel.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/chassis/controller/chassisControllerBuilder.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "typhoon/imuOdometry.hpp"
#include <memory>

using namespace okapi;

IMU imu(15);

TimeUtil timeUtil = TimeUtilFactory::withSettledUtilParams(10, 1, 250_ms);

std::shared_ptr<ChassisController> controller = 
	ChassisControllerBuilder()
		.withMotors({-2, -4}, {11, 12})
		.withDimensions(AbstractMotor::gearset::blue, {{2.75_in, 10.8_in}, int(imev5BlueTPR / 0.75)})
		.withGains(
			{0.001, 0.0002, 0.00001}, // Distance controller gains
			{0.001, 0.0002, 0.00001}, // Turn controller gains
			{0.001, 0, 0}  // Angle controller gains (helps drive straight)
		)
		.withDerivativeFilters(
			std::make_unique<AverageFilter<3>>(), // Distance controller filter
			std::make_unique<AverageFilter<3>>(), // Turn controller filter
			std::make_unique<AverageFilter<3>>()  // Angle controller filter
		)
		.build();

std::shared_ptr<Odometry> odometry{
	new IMUOdometry(timeUtil,
		controller->getModel(),
		imu,
		controller->getChassisScales())};

std::shared_ptr<OdomChassisController> chassis{
	new DefaultOdomChassisController(timeUtil, odometry, controller)};

// std::shared_ptr<AsyncMotionProfileController> profileController =
//   AsyncMotionProfileControllerBuilder()
//     .withLimits({
//       1.0, // Maximum linear velocity of the Chassis in m/s
//       2.0, // Maximum linear acceleration of the Chassis in m/s/s
//       10.0 // Maximum linear jerk of the Chassis in m/s/s/s
//     })
//     .withOutput(chassis)
//     .buildMotionProfileController();


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
	for (int i = 0; i < 4; i++) {
		chassis->moveDistance(1_ft);
		chassis->turnAngle(90_deg);
	}
}

void opcontrol() {
	Controller primary;
	ControllerButton runAuto(ControllerDigital::A);
	// ControllerButton runOuttake(ControllerDigital::R1);
	// ControllerButton runIntake(ControllerDigital::R2);

	chassis->startOdomThread();

	while (true) {
		chassis->getModel()->curvature(
				primary.getAnalog(ControllerAnalog::leftY),
				primary.getAnalog(ControllerAnalog::rightX));
		if (runAuto.changedToPressed()) {
			autonomous();
		}

		OdomState state = chassis->getState();
		pros::lcd::print(0, "%f", state.x.convert(foot));
		pros::lcd::print(1, "%f", state.y.convert(foot));
		pros::lcd::print(2, "%f", state.theta.convert(degree));
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