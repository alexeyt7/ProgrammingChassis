#include "main.h"

#include <memory>
#include <sstream>

#include "replay.hpp"

using namespace okapi;

const double MAX_VOLTAGE = 12000;  // millivolts

const int32_t FLYWHEEL_TARGET_LOW = 410;
const int32_t FLYWHEEL_TARGET_HIGH = 500;

const int32_t BLUE_HUE = 200;
const int32_t RED_HUE = 0;

bool isCompetition = false;
AllianceColor color = AllianceColor::RED;

IMU imu{13};
DistanceSensor leftDistance{11};
DistanceSensor rightDistance{20};
Potentiometer kicker{'A'};

TimeUtil timeUtil = TimeUtilFactory::withSettledUtilParams(1, 1, 250_ms);

MotorGroup flywheel{-4, 5};
MotorGroup indexer{3};
MotorGroup intake{-2};

MotorGroup leftDrive{-6, -7};
MotorGroup rightDrive{8, 9};
OpticalSensor colorSensor{10};

Recording recording{};

auto chassis =
    ChassisControllerBuilder()
        .withMotors(leftDrive, rightDrive)
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

void resetIndexer() {
	while (kicker.get() > 3120) {
		indexer.moveVoltage(MAX_VOLTAGE * 0.3);
		pros::delay(20);
	}
	while (kicker.get() < 3120) {
		indexer.moveVoltage(MAX_VOLTAGE * -0.3);
		pros::delay(20);
	}
	indexer.moveVoltage(0);
}

int angleDifferenceAbs(int angle1, int angle2) {
	int difference = angle1 - angle2;
	while (difference > 180) {
		difference -= 360;
	}
	while (difference < -180) {
		difference += 360;
	}
	return abs(difference);
}

AllianceColor rollerColor() {
	int hue = colorSensor.getHue();
	int distRed = angleDifferenceAbs(RED_HUE, hue);
	int distBlue = angleDifferenceAbs(BLUE_HUE, hue);
	return distRed < distBlue ? AllianceColor::RED : AllianceColor::BLUE;
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::print(0, "Initializing, please wait...");
	imu.reset();
	uint32_t calibrationStart = pros::millis();
	flywheel.setGearing(AbstractMotor::gearset::blue);
	flywheel.setBrakeMode(AbstractMotor::brakeMode::coast);
	indexer.setBrakeMode(AbstractMotor::brakeMode::brake);

	pros::Task::delay_until(&calibrationStart, 2000);
	resetIndexer();
	pros::lcd::clear_line(0);
	pros::lcd::print(0, "Initialization finished!");
}

void disabled() {}
void competition_initialize() {
	isCompetition = true;
}
void autonomous() {}

pros::Task kickDisk{[] {
	while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
		while (kicker.get() > 2500) {
			indexer.moveVoltage(MAX_VOLTAGE * 0.3);
			pros::delay(20);
		}
		while (kicker.get() < 3120) {
			indexer.moveVoltage(MAX_VOLTAGE * -0.3);
			pros::delay(20);
		}

		indexer.moveVoltage(0);
		pros::delay(1000);
	}
}};

// pros::Task setRoller {
// 	[] {}
// }

void applyInputs(Inputs& input) {
	chassis->getModel()->curvature(input.thrust, input.turn, 0.01);
	if (input.flywheel > 0) {
		flywheel.moveVelocity(input.flywheel);
	} else {
		flywheel.moveVoltage(0);
	}
	if (input.flywheel != 0 && flywheel.getActualVelocity() > input.flywheel) {
		kickDisk.notify();
	}
	intake.moveVoltage(MAX_VOLTAGE * input.intake);
}

// void playRecording() {
// 	for (int i = 0; i < recording.length(); i++) {
// 		uint32_t loopStart = pros::millis();

// 		Inputs u = recording.getFrame(i, {0, 0, 0});
// 		applyInputs(u);
// 		pros::Task::delay_until(&loopStart, 50);
// 	}
// }

void opcontrol() {
	Controller primary{};
	bool isRecording = false;
	bool recordingFinished = false;

	while (true) {
		uint32_t loopStart = pros::millis();
		Inputs input{};
		bool shift = primary.getDigital(ControllerDigital::L1);

		if (!isCompetition) {
			if (!isRecording && primary.getDigital(ControllerDigital::B)) {
				// recording.clear();
				isRecording = true;
				primary.setText(0, 0, "recording!");
			}
			if (primary.getDigital(ControllerDigital::Y)) {
				primary.setText(0, 0, "building! ");
				isRecording = false;
				// recording.buildModel();
				primary.setText(0, 0, "stopped!  ");
			}
			if (primary.getDigital(ControllerDigital::A)) {
				primary.setText(0, 0, "replaying!  ");
				// playRecording();
				primary.setText(0, 0, "finished!  ");
			}
		}

		input.thrust = primary.getAnalog(ControllerAnalog::leftY);
		input.turn = primary.getAnalog(ControllerAnalog::rightX);

		if (primary.getDigital(ControllerDigital::R2)) {
			input.flywheel = shift ? FLYWHEEL_TARGET_HIGH : FLYWHEEL_TARGET_LOW;
		}
		if (primary.getDigital(ControllerDigital::L2)) {
			input.intake = shift ? -1 : 1;
		}

		if (primary.getDigital(ControllerDigital::X)) {
			kickDisk.notify();
		}

		applyInputs(input);

		if (rollerColor() == AllianceColor::RED) {
			primary.setText(0, 0, "RED ");
		} else {
			primary.setText(0, 0, "BLUE");
		}
		pros::lcd::print(1, "%f %f", leftDistance.get(), rightDistance.get());

		if (isRecording) {
			recording.addFrame(input,
			                   {leftDistance.get(), rightDistance.get(), imu.get()});
		}
		pros::lcd::print(2, "%f", kicker.get());
		pros::lcd::print(3, "%f", flywheel.getActualVelocity() * 6);

		pros::Task::delay_until(&loopStart, 50);
	}
}
