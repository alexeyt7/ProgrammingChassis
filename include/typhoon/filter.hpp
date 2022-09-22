#include <functional>

#include "eigen.h"
#include "main.h"

#define STATES 5
#define WEIGHTS 2 * STATES + 1
#define INPUTS 2
#define OUTPUTS 4

using StateVector = Vectord<STATES>;
using InputVector = Vectord<INPUTS>;
using OutputVector = Vectord<OUTPUTS>;
using WeightVector = Vectord<WEIGHTS>;

namespace filter {
class PositionFilter {
	public:
	PositionFilter(StateVector mean0, Matrixd<STATES, STATES> variance0,
	               Matrixd<OUTPUTS, OUTPUTS> measurementNoise);
	void step(InputVector input, OutputVector output);
	void predict(InputVector input);
	void update(OutputVector output);

	private:
	const Matrixd<OUTPUTS, OUTPUTS> measurementNoise_;
	StateVector xMean_;
	Matrixd<STATES, STATES> xVariance_;
	Matrixd<STATES, WEIGHTS> xSigmas_;
	WeightVector meanWeights_;
	WeightVector varianceWeights_;
	std::function<StateVector(StateVector, InputVector)> modelFunction_;
	std::function<StateVector(Matrixd<STATES, WEIGHTS>, WeightVector)>
	    xMeanFunction_;
	std::function<StateVector(StateVector, StateVector)> xResidualFunction_;
	std::function<OutputVector(StateVector)> measurementFunction_;
	std::function<OutputVector(Matrixd<OUTPUTS, WEIGHTS>, WeightVector)>
	    zMeanFunction_;
	std::function<OutputVector(OutputVector, OutputVector)> zResidualFunction_;
};
}  // namespace filter
