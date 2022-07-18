#include <map>

#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/api/control/util/pathfinderUtil.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QArea.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/squiggles/squiggles.hpp"

using namespace okapi;

class RamseteController : public AsyncMotionProfileController {
	public:
	RamseteController(
	    const TimeUtil& itimeUtil, const PathfinderLimits& ilimits,
	    const std::shared_ptr<ChassisModel>& imodel, const ChassisScales& iscales,
	    const AbstractMotor::GearsetRatioPair& ipair,
	    std::shared_ptr<Odometry> iodometry, double ib, double izeta,
	    const std::shared_ptr<Logger>& ilogger = Logger::getDefaultLogger());

	void executeSinglePath(const std::vector<squiggles::ProfilePoint>& path,
	                       std::unique_ptr<AbstractRate> rate) override;

	protected:
	std::shared_ptr<Odometry> odometry;
	const double b;
	const double zeta;

	private:
	double wheelSpeedsToLinearVelocity(double left, double right);
	double wheelSpeedsToAngularVelocity(double left, double right);
	OdomState poseToState(squiggles::Pose pose);
};
