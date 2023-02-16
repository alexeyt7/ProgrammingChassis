#include "eigen.h"
#include "main.h"
#include "okapi/api/units/QTime.hpp"

/*
auto system: replay a set of inputs from a driver with sensor correction.to do
this, construct a mapping from change in sensor reading to inputs, and use it to
calculate an input to add to the previously recorded one. this mapping can be
constructed once and stored, saving processing time. through this, the robot
"learns" what targets look like and can gain benefits of global state tracking
without the complexity or tuning.

y: readings
u: inputs
*/

using namespace okapi;

const QTime FRAME_TIME_MS = 50_ms;
const int INPUTS = 5;
const int NOISY_INPUTS = 2;
const int OUTPUTS = 3;

using InputVector = Vectord<INPUTS>;
using OutputVector = Vectord<OUTPUTS>;

class Recording {
	private:
	std::vector<Inputs> inputs_{};
	std::vector<OutputVector> outputs_{};
	// Matrixd<INPUTS, OUTPUTS> model_ = Matrixd<INPUTS, OUTPUTS>::Zero();

	public:
	int length();
	void clear();
	void addFrame(Inputs input, OutputVector output);
	// void buildModel();
	// Matrixd<1, OUTPUTS> regress(std::vector<double> input,
	//                             std::vector<OutputVector> outputs);

	Inputs getFrame(int i, OutputVector y);
	void write(std::string path);
	void read(std::string path);
};
