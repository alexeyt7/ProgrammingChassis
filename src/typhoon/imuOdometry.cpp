/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "typhoon/imuOdometry.hpp"

#include <cmath>

#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/util/mathUtil.hpp"

using namespace okapi;

IMUOdometry::IMUOdometry(const TimeUtil& itimeUtil,
                         const std::shared_ptr<ReadOnlyChassisModel>& imodel,
                         const IMU& iimu, const ChassisScales& ichassisScales,
                         const std::shared_ptr<Logger>& ilogger)
    : logger(ilogger),
      rate(itimeUtil.getRate()),
      timer(itimeUtil.getTimer()),
      model(imodel),
      chassisScales(ichassisScales),
      imu(iimu) {}

void IMUOdometry::setScales(const ChassisScales& ichassisScales) {
	chassisScales = ichassisScales;
}

void IMUOdometry::step() {
	const auto deltaT = timer->getDt();

	if (deltaT.getValue() != 0) {
		newTicks = model->getSensorVals();
		tickDiff = newTicks - lastTicks;
		lastTicks = newTicks;

		QAngle newTheta = imu.get() * degree;
		QAngle dTheta = newTheta - lastTheta;
		lastTheta = newTheta;

		const auto newState = odomMathStep(tickDiff, dTheta, deltaT);

		state.x += newState.x;
		state.y += newState.y;
		state.theta += newState.theta;
	}
}

OdomState IMUOdometry::odomMathStep(
    const std::valarray<std::int32_t>& itickDiff, const QAngle& idTheta,
    const QTime&) {
	if (itickDiff.size() < 2) {
		LOG_ERROR_S("IMUOdometry: itickDiff did not have at least two elements.");
		return OdomState{};
	}

	for (auto&& elem : itickDiff) {
		if (std::abs(elem) > maximumTickDiff) {
			LOG_ERROR("IMUOdometry: A tick diff (" + std::to_string(elem) +
			          ") was greater than the maximum allowable diff (" +
			          std::to_string(maximumTickDiff) +
			          "). Skipping this odometry step.");
			return OdomState{};
		}
	}

	const double deltaL = itickDiff[0] / chassisScales.straight;
	const double deltaR = itickDiff[1] / chassisScales.straight;

	double deltaTheta = idTheta.convert(radian);
	double localOffX, localOffY;

	if (deltaTheta != 0) {
		localOffX = 2 * std::sin(deltaTheta / 2) *
		            chassisScales.middleWheelDistance.convert(meter);
		localOffY =
		    2 * std::sin(deltaTheta / 2) *
		    (deltaR / deltaTheta + chassisScales.wheelTrack.convert(meter) / 2);
	} else {
		localOffX = 0;
		localOffY = deltaR;
	}

	double avgA = state.theta.convert(radian) + (deltaTheta / 2);

	double polarR = std::sqrt(localOffX * localOffX + localOffY * localOffY);
	double polarA = std::atan2(localOffY, localOffX) - avgA;

	double dX = std::sin(polarA) * polarR;
	double dY = std::cos(polarA) * polarR;

	if (isnan(dX)) {
		dX = 0;
	}

	if (isnan(dY)) {
		dY = 0;
	}

	if (isnan(deltaTheta)) {
		deltaTheta = 0;
	}

	return OdomState{dX * meter, dY * meter, deltaTheta * radian};
}

OdomState IMUOdometry::getState(const StateMode& imode) const {
	if (imode == StateMode::FRAME_TRANSFORMATION) {
		return state;
	} else {
		return OdomState{state.y, state.x, state.theta};
	}
}

void IMUOdometry::setState(const OdomState& istate, const StateMode& imode) {
	LOG_DEBUG("State set to: " + istate.str());
	if (imode == StateMode::FRAME_TRANSFORMATION) {
		state = istate;
	} else {
		state = OdomState{istate.y, istate.x, istate.theta};
	}
}

std::shared_ptr<ReadOnlyChassisModel> IMUOdometry::getModel() {
	return model;
}

ChassisScales IMUOdometry::getScales() {
	return chassisScales;
}
