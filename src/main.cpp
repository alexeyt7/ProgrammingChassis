#include "main.h"

#include <memory>

#include "typhoon/imuOdometry.hpp"
#include "typhoon/ramseteController.hpp"

using namespace okapi;

IMU imu(16);
MotorGroup intake{-5, 15};
MotorGroup catapult{-4, 14};
ADIButton catapultLimit{'A'};

TimeUtil timeUtil = TimeUtilFactory::withSettledUtilParams(1, 1, 250_ms);

auto controller =
    ChassisControllerBuilder()
        .withMotors({-2, -3}, {12, 13})
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

pros::Task launchCatapult{[] {
	while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
		while (catapultLimit.isPressed()) {
			catapult.moveVoltage(12000);
		}
		catapult.moveVoltage(0);
	}
}};

void initialize() {
	pros::lcd::initialize();
	pros::lcd::print(0, "Initializing, please wait...");
	imu.reset();
	uint32_t calibrationStart = pros::millis();
	ramseteController->generatePath({{0_in, 0_ft, 0_deg}, {4_in, 0_ft, 0_deg}},
	                                "fwd");
	chassis->startOdomThread();
	ramseteController->startThread();

	catapult.setBrakeMode(AbstractMotor::brakeMode::brake);
	while (!catapultLimit.isPressed()) {
		catapult.moveVoltage(12000);
	}
	catapult.moveVoltage(0);

	pros::Task::delay_until(&calibrationStart, 2000);
	pros::lcd::clear_line(0);
}

void disabled() {}
void competition_initialize() {}
void autonomous() {
#if far
	launchCatapult.notify();
	pros::delay(2000);
	// ramseteController->setTarget("fwd", true);
	// ramseteController->waitUntilSettled();
	chassis->getChassisController()->moveDistance(24_in);
	chassis->getChassisController()->turnAngle(100_deg);
	chassis->getChassisController()->moveDistanceAsync(4_in);
	pros::delay(500);

	intake.moveRelative(90, 60);
	pros::delay(1000);
	chassis->getChassisController()->moveDistance(-4_in);
#else
	chassis->getChassisController()->moveDistanceAsync(4_in);
	pros::delay(500);

	intake.moveRelative(90, 60);
	pros::delay(1000);

	chassis->getChassisController()->moveDistance(-4_in);
	chassis->getChassisController()->turnAngle(100_deg);

	chassis->getChassisController()->moveDistance(-48_in);

	launchCatapult.notify();
	pros::delay(2000);
#endif
}

void opcontrol() {
	Controller primary;

	intake.tarePosition();

	while (true) {
		chassis->getModel()->curvature(primary.getAnalog(ControllerAnalog::leftY),
		                               primary.getAnalog(ControllerAnalog::rightX));
		if (primary.getDigital(ControllerDigital::A)) {
			autonomous();
		}

		pros::lcd::print(3, "intake: %5.f", intake.getPosition());

		OdomState state = chassis->getState();
		pros::lcd::print(0, "%f", state.x.convert(foot));
		pros::lcd::print(1, "%f", state.y.convert(foot));
		pros::lcd::print(2, "%f", state.theta.convert(degree));
		if (primary.getDigital(ControllerDigital::R2)) {
			intake.moveVoltage(12000 * 0.7);
		} else if (primary.getDigital(ControllerDigital::R1)) {
			intake.moveVoltage(-12000 * 0.7);
		} else {
			intake.moveVoltage(0);
		}

		if (primary.getDigital(ControllerDigital::L1)) {
			launchCatapult.notify();
		}

		if (!catapultLimit.isPressed()) {
			catapult.moveVoltage(12000);
		} else {
			catapult.moveVoltage(0);
		}
		pros::delay(20);
	}
}
