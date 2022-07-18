#include "main.h"

#include <memory>

#include "typhoon/imuOdometry.hpp"
#include "typhoon/ramseteController.hpp"

using namespace okapi;

IMU imu(15);

TimeUtil timeUtil = TimeUtilFactory::withSettledUtilParams(1, 1, 250_ms);

auto controller =
    ChassisControllerBuilder()
        .withMotors({-2, -4}, {11, 12})
        .withDimensions(AbstractMotor::gearset::blue,
                        {{2.75_in, 10.8_in}, int(imev5BlueTPR / 0.75)})
        .withGains(
            {0.001, 0.0002, 0.00001},  // Distance controller gains
            {0.001, 0.0002, 0.00001},  // Turn controller gains
            {0.001, 0, 0}  // Angle controller gains (helps drive straight)
            )
        .withDerivativeFilters(
            std::make_unique<AverageFilter<3>>(),  // Distance controller filter
            std::make_unique<AverageFilter<3>>(),  // Turn controller filter
            std::make_unique<AverageFilter<3>>()   // Angle controller filter
            )
        .build();

auto odometry = std::make_shared<IMUOdometry>(
    timeUtil, controller->getModel(), imu, controller->getChassisScales());

auto chassis = std::make_shared<DefaultOdomChassisController>(
    timeUtil, odometry, controller);

auto ramseteController = std::make_shared<RamseteController>(
    timeUtil, PathfinderLimits{1, 2, 10}, controller->getModel(),
    controller->getChassisScales(), controller->getGearsetRatioPair(), odometry,
    2, 0.7);

void initialize() {
	pros::lcd::initialize();
	pros::lcd::print(0, "Initializing, please wait...");
	imu.reset();
	uint32_t calibrationStart = pros::millis();
	pros::lcd::initialize();
	ramseteController->generatePath({{0_ft, 0_ft, 0_deg}, {2_ft, -1_ft, -90_deg}},
	                                "A");
	chassis->startOdomThread();
	ramseteController->startThread();
	pros::Task::delay_until(&calibrationStart, 2000);
	pros::lcd::clear_line(0);
}

void disabled() {}
void competition_initialize() {}
void autonomous() {
	ramseteController->setTarget("A");
	ramseteController->waitUntilSettled();
}

void opcontrol() {
	Controller primary;
	ControllerButton runAuto(ControllerDigital::A);
	// ControllerButton runOuttake(ControllerDigital::R1);
	// ControllerButton runIntake(ControllerDigital::R2);

	while (true) {
		chassis->getModel()->curvature(primary.getAnalog(ControllerAnalog::leftY),
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
