#include "typhoon/ramseteController.hpp"

#include "geometry/pose.hpp"

/**
 * Taken from wpilib, helps with small errors in theta
 * Returns sin(x) / x.
 *
 * @param x Value of which to take sinc(x).
 */
double Sinc(double x) {
	if (fabs(x) < 1e-9) {
		return 1.0 - 1.0 / 6.0 * x * x;
	} else {
		return sin(x) / x;
	}
}

RamseteController::RamseteController(
    const TimeUtil& itimeUtil, const PathfinderLimits& ilimits,
    const std::shared_ptr<ChassisModel>& imodel, const ChassisScales& iscales,
    const AbstractMotor::GearsetRatioPair& ipair,
    std::shared_ptr<Odometry> iodometry, double ib, double izeta,
    const std::shared_ptr<Logger>& ilogger)
    : b(ib),
      zeta(izeta),
      odometry(iodometry),
      AsyncMotionProfileController(itimeUtil, ilimits, imodel, iscales, ipair,
                                   ilogger) {}

void RamseteController::executeSinglePath(
    const std::vector<squiggles::ProfilePoint>& path,
    std::unique_ptr<AbstractRate> rate) {
	const int reversed = direction.load(std::memory_order_acquire);
	const bool followMirrored = mirrored.load(std::memory_order_acquire);

	currentPathMutex.lock();
	// store this locally so we aren't accessing the path when we don't know if
	// it's valid
	std::size_t pathSize = path.size();
	odometry->setState(poseToState(path[0].vector.pose));
	currentPathMutex.unlock();

	for (std::size_t i = 0; i < pathSize && !isDisabled(); ++i) {
		// This mutex is used to combat an edge case of an edge case
		// if a running path is asked to be removed at the moment this loop is
		// executing
		std::scoped_lock lock(currentPathMutex);

		const auto segDT = DT * second;

		double v_d, w_d;
		{
			double left = path[i].wheel_velocities[0];
			double right = path[i].wheel_velocities[1];

			v_d = wheelSpeedsToLinearVelocity(left, right);
			w_d = wheelSpeedsToAngularVelocity(left, right);
		}

		double k = 2 * zeta * sqrt(pow(w_d, 2) + b * pow(v_d, 2));

		auto state = odometry->getState();
		auto targetState = poseToState(path[i].vector.pose);

		double errorXGlobal = (targetState.y - state.x).convert(meter);
		double errorYGlobal = (targetState.x - state.y).convert(meter);
		double errorTheta = (targetState.theta - state.theta).convert(radian);
		/* constrain to (-M_PI, M_PI) to prevent large turns */
		while (errorTheta < -M_PI) {
			errorTheta += 2 * M_PI;
		}
		while (errorTheta > M_PI) {
			errorTheta -= 2 * M_PI;
		}

		double theta = state.theta.convert(radian);
		/* convert error to robot reference frame */
		double errorX = cos(theta) * errorXGlobal + sin(theta) * errorYGlobal;
		double errorY = -sin(theta) * errorXGlobal + cos(theta) * errorYGlobal;

		double v = v_d * cos(errorTheta) + k * errorX;
		double w = w_d + k * errorTheta + b * v_d * Sinc(errorTheta) * errorY;

		double forward = v;
		double turn = w * scales.wheelTrack.convert(meter);
		const auto leftRPM =
		    convertLinearToRotational((forward + turn) * mps).convert(rpm);
		const auto rightRPM =
		    convertLinearToRotational((forward - turn) * mps).convert(rpm);

		const double rightSpeed =
		    rightRPM / toUnderlyingType(pair.internalGearset) * reversed;
		const double leftSpeed =
		    leftRPM / toUnderlyingType(pair.internalGearset) * reversed;

		if (followMirrored) {
			model->left(rightSpeed);
			model->right(leftSpeed);
		} else {
			model->left(leftSpeed);
			model->right(rightSpeed);
		}

		// Unlock before the delay to be nice to other tasks
		currentPathMutex.unlock();

		rate->delayUntil(segDT);
	}
}

double RamseteController::wheelSpeedsToLinearVelocity(double left,
                                                      double right) {
	return 0.5 * (left + right);
}

double RamseteController::wheelSpeedsToAngularVelocity(double left,
                                                       double right) {
	return 0.5 * (left - right) / scales.wheelTrack.convert(meter);
}

OdomState RamseteController::poseToState(squiggles::Pose pose) {
	return OdomState{pose.x * meter, pose.y * meter,
	                 (M_PI_2 - pose.yaw) * radian};
}
